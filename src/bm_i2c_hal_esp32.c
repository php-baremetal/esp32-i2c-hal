/*
   +----------------------------------------------------------------------+
   | php-baremetal: esp32-i2c-hal                                         |
   +----------------------------------------------------------------------+
   | Copyright (c) 2026 Gianfrancesco Aurecchia                           |
   +----------------------------------------------------------------------+
   | This source file is subject to the MIT license, that is bundled with |
   | this package in the file LICENSE.                                    |
   +----------------------------------------------------------------------+
   | Authors: Gianfrancesco Aurecchia <gianfri@php-baremetal.com>         |
   +----------------------------------------------------------------------+
*/

/* ESP32 implementation of the i2c HAL (bm_i2c_hal.h) over esp_driver_i2c, FreeRTOS and the executor. */
#include "bm_i2c_hal.h"
#include "bm_events_hal.h"
#include "executor.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "soc/soc_caps.h"

#include <stdlib.h>
#include <string.h>

static const char *TAG = "bm_i2c";

#define SCAN_MAX_TIMEOUTS  8
#define DEV_TIMEOUT_MS     1000
#define PROBE_TIMEOUT_MS   50
#define ENQUEUE_TIMEOUT_MS 1000

typedef struct {
    i2c_master_bus_handle_t handle;
    SemaphoreHandle_t       lock;
} esp_i2c_bus_t;

static esp_i2c_bus_t *hal_bus(i2c_bus_t *bus)
{
    return (esp_i2c_bus_t *) bus->hal;
}

static bm_err_t bus_open_run(i2c_bus_t *bus)
{
    esp_i2c_bus_t *b = calloc(1, sizeof(*b));
    if (!b) {
        return BM_ERR_NO_MEM;
    }
    b->lock = xSemaphoreCreateMutex();
    if (!b->lock) {
        free(b);
        return BM_ERR_NO_MEM;
    }
    i2c_master_bus_config_t cfg = {
        .i2c_port = bus->port,
        .sda_io_num = bus->sda,
        .scl_io_num = bus->scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t e = i2c_new_master_bus(&cfg, &b->handle);
    if (e != ESP_OK) {
        vSemaphoreDelete(b->lock);
        free(b);
        return e;
    }
    e = i2c_master_bus_reset(b->handle);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "bus reset after open failed: %s", esp_err_to_name(e));
    }
    if (bus->port < 0) {
        /* Auto picks the first free HP port, so every lower port is occupied and the walk logs nothing. */
        for (int p = 0; p < SOC_HP_I2C_NUM; p++) {
            i2c_master_bus_handle_t h;
            if (i2c_master_get_bus_handle(p, &h) == ESP_OK && h == b->handle) {
                bus->port = p;
                break;
            }
        }
    }
    bus->hal = b;
    return BM_OK;
}

static esp_err_t bus_open_tramp(void *arg)
{
    return bus_open_run(arg);
}

/* An executor-owned bus is opened on the executor task so the I2C interrupt lands on its core. */
static esp_err_t on_owner(i2c_bus_t *bus, esp_err_t (*fn)(void *), void *arg)
{
    if (bus->owner == I2C_OWNER_CORE1 && !executor_on_task()) {
        return executor_run_sync(fn, arg, ENQUEUE_TIMEOUT_MS);
    }
    return fn(arg);
}

bm_err_t bm_i2c_hal_bus_open(i2c_bus_t *bus)
{
    return on_owner(bus, bus_open_tramp, bus);
}

static esp_err_t bus_close_run(void *arg)
{
    i2c_bus_t *bus = arg;
    esp_i2c_bus_t *b = hal_bus(bus);
    if (b) {
        i2c_del_master_bus(b->handle);
        vSemaphoreDelete(b->lock);
        free(b);
        bus->hal = NULL;
    }
    return ESP_OK;
}

void bm_i2c_hal_bus_close(i2c_bus_t *bus)
{
    on_owner(bus, bus_close_run, bus);
}

static esp_err_t dev_open_run(void *arg)
{
    i2c_dev_t *d = arg;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = d->addr,
        .scl_speed_hz = d->hz,
    };
    i2c_master_dev_handle_t handle;
    esp_err_t e = i2c_master_bus_add_device(hal_bus(d->bus)->handle, &cfg, &handle);
    if (e != ESP_OK) {
        return e;
    }
    d->hal = handle;
    return BM_OK;
}

bm_err_t bm_i2c_hal_dev_open(i2c_dev_t *d)
{
    return on_owner(d->bus, dev_open_run, d);
}

static esp_err_t dev_close_run(void *arg)
{
    i2c_dev_t *d = arg;
    if (d->hal) {
        i2c_master_bus_rm_device((i2c_master_dev_handle_t) d->hal);
        d->hal = NULL;
    }
    return ESP_OK;
}

void bm_i2c_hal_dev_close(i2c_dev_t *d)
{
    on_owner(d->bus, dev_close_run, d);
}

typedef struct {
    i2c_dev_t          *d;
    const bm_i2c_txn_t *t;
} txn_ctx_t;

/* The wire op, unlocked: the caller holds the lock for SYNC, the executor is sole owner for CORE1. */
static esp_err_t txn_once(const txn_ctx_t *c)
{
    const bm_i2c_txn_t *t = c->t;
    i2c_master_dev_handle_t h = (i2c_master_dev_handle_t) c->d->hal;
    switch (t->op) {
    case BM_I2C_READ:
        return i2c_master_receive(h, t->rbuf, t->n, DEV_TIMEOUT_MS);
    case BM_I2C_WRITE:
        return i2c_master_transmit(h, t->wbuf, t->n, DEV_TIMEOUT_MS);
    case BM_I2C_READ_REG:
        return i2c_master_transmit_receive(h, &t->reg, 1, t->rbuf, t->n, DEV_TIMEOUT_MS);
    case BM_I2C_WRITE_REG: {
        uint8_t stack[1 + 32];
        uint8_t *p = stack;
        if (1 + t->n > sizeof(stack)) {
            p = malloc(1 + t->n);
            if (!p) {
                return BM_ERR_NO_MEM;
            }
        }
        p[0] = t->reg;
        memcpy(p + 1, t->wbuf, t->n);
        esp_err_t e = i2c_master_transmit(h, p, 1 + t->n, DEV_TIMEOUT_MS);
        if (p != stack) {
            free(p);
        }
        return e;
    }
    case BM_I2C_WRITE_REG1: {
        uint8_t b[2] = { t->reg, t->val };
        return i2c_master_transmit(h, b, 2, DEV_TIMEOUT_MS);
    }
    case BM_I2C_PROBE:
        return i2c_master_probe(hal_bus(c->d->bus)->handle, c->d->addr, PROBE_TIMEOUT_MS);
    }
    return ESP_ERR_INVALID_ARG;
}

static bool bus_stuck(esp_err_t e)
{
    return e == ESP_ERR_TIMEOUT || e == ESP_ERR_INVALID_STATE;
}

/* A timeout or invalid state resets the bus once and retries the transaction once. */
static esp_err_t txn_run(void *arg)
{
    const txn_ctx_t *c = arg;
    esp_err_t e = txn_once(c);
    if (bus_stuck(e) && c->t->op != BM_I2C_PROBE) {
        esp_err_t r = i2c_master_bus_reset(hal_bus(c->d->bus)->handle);
        ESP_LOGW(TAG, "txn %s, bus reset %s, retrying", esp_err_to_name(e), esp_err_to_name(r));
        e = txn_once(c);
    }
    return e;
}

/* Route by ownership: CORE1 to the executor (inline if already on it), SYNC under the bus lock. */
static esp_err_t route(i2c_bus_t *bus, esp_err_t (*fn)(void *), void *arg)
{
    if (bus->owner == I2C_OWNER_CORE1) {
        if (executor_on_task()) {
            return fn(arg);
        }
        return executor_run_sync(fn, arg, ENQUEUE_TIMEOUT_MS);
    }
    esp_i2c_bus_t *b = hal_bus(bus);
    xSemaphoreTake(b->lock, portMAX_DELAY);
    esp_err_t e = fn(arg);
    xSemaphoreGive(b->lock);
    return e;
}

bm_err_t bm_i2c_hal_transfer(i2c_dev_t *d, const bm_i2c_txn_t *t)
{
    txn_ctx_t c = { .d = d, .t = t };
    return route(d->bus, txn_run, &c);
}

typedef struct {
    i2c_bus_t *bus;
    uint8_t    first;
    uint8_t    last;
    uint8_t   *bits;
} scan_ctx_t;

static esp_err_t scan_run(void *arg)
{
    scan_ctx_t *c = arg;
    int timeouts = 0;
    for (uint16_t a = c->first; a <= c->last; a++) {
        esp_err_t e = i2c_master_probe(hal_bus(c->bus)->handle, a, PROBE_TIMEOUT_MS);
        if (e == ESP_OK) {
            c->bits[a >> 3] |= (uint8_t) (1u << (a & 7));
            timeouts = 0;
        } else if (e == ESP_ERR_TIMEOUT) {
            if (++timeouts >= SCAN_MAX_TIMEOUTS) {
                i2c_master_bus_reset(hal_bus(c->bus)->handle);
                return ESP_ERR_TIMEOUT;
            }
        } else {
            timeouts = 0;
        }
    }
    return ESP_OK;
}

bm_err_t bm_i2c_hal_scan(i2c_bus_t *bus, uint8_t first, uint8_t last, uint8_t bitmap[16])
{
    scan_ctx_t c = { .bus = bus, .first = first, .last = last, .bits = bitmap };
    return route(bus, scan_run, &c);
}

/* The executor calls this on its task; the driver's sample does its own routing. */
static esp_err_t poll_sample(void *dev, void *out)
{
    i2c_dev_t *d = dev;
    return d->driver->poll.sample(d, out);
}

void *bm_i2c_hal_poll_start(i2c_dev_t *d, uint32_t hz, size_t depth)
{
    return executor_poll(poll_sample, d, d->driver->poll.sample_size, hz, depth);
}

void bm_i2c_hal_poll_on_sample(void *poller, void (*notify)(void *arg), void *arg)
{
    executor_poll_on_sample(poller, notify, arg);
}

void bm_i2c_hal_poll_pause(void *poller, bool paused)
{
    executor_poll_pause(poller, paused);
}

bool bm_i2c_hal_poll_latest(void *poller, void *out)
{
    return executor_poll_latest(poller, out);
}

size_t bm_i2c_hal_poll_drain(void *poller, void *out, size_t max_samples)
{
    return executor_poll_drain(poller, out, max_samples);
}

/* Interrupt service: one GPIO edge handler per device, kicking an executor on-demand job. */
typedef struct {
    executor_irq_t *job;
    i2c_dev_t      *d;
    int             pin;
} irq_handle_t;

/* The executor runs this on its task; the driver's service reads the chip through the HAL. The result is only
 * logged. */
static esp_err_t irq_service(void *arg)
{
    irq_handle_t *h = arg;
    /* The service may stop its own handle (bm_i2c_hal_irq_stop runs inline there and frees h): copy first. */
    i2c_dev_t *d = h->d;
    int pin = h->pin;
    bm_err_t e = d->driver->irq.service(d);
    if (e != BM_OK) {
        ESP_LOGW(TAG, "irq service on pin %d returned %s", pin, bm_i2c_hal_err_name(e));
    }
    return ESP_OK;
}

/* Interrupt context, IRAM: it only kicks the job (executor_irq_kick_from_isr is IRAM_ATTR). */
static void IRAM_ATTR irq_isr(void *arg)
{
    /* gpio_isr_handler_remove clears fn and args without the ISR taking its lock, so an edge on the other core can
     * still be called with args == NULL. */
    irq_handle_t *h = arg;
    if (h) {
        executor_irq_kick_from_isr(h->job);
    }
}

void *bm_i2c_hal_irq_start(i2c_dev_t *d, int pin)
{
    if (!d || !d->driver || !d->driver->irq.supported || !d->driver->irq.service || pin < 0 || pin >= GPIO_NUM_MAX || !GPIO_IS_VALID_GPIO(pin)) {
        return NULL;
    }
    irq_handle_t *h = calloc(1, sizeof(*h));
    if (!h) {
        return NULL;
    }
    h->d = d;
    h->pin = pin;
    h->job = executor_irq(irq_service, h);
    if (!h->job) {
        free(h);
        return NULL;
    }
    /* The events HAL configures the pin (input, pull-up, falling edge) and installs the GPIO ISR service in one
     * place, so a firmware that also uses another GPIO interrupt (ext-display's TE) does not install it twice.
     * It returns void, so a failure there cannot be seen here. Both this and stop run on the PHP task, which is
     * what the HAL's install-once flag relies on. */
    bm_hal_gpio_on_falling(pin, irq_isr, h);
    return h;
}

void bm_i2c_hal_irq_after(void *h, uint32_t ms)
{
    executor_irq_after(((irq_handle_t *) h)->job, ms);
}

void bm_i2c_hal_irq_pause(void *h, bool paused)
{
    executor_irq_pause(((irq_handle_t *) h)->job, paused);
}

void bm_i2c_hal_irq_stop(void *h)
{
    irq_handle_t *ih = h;
    if (!ih) {
        return;
    }
    /* The handler goes first: the job must not be kicked once it is freed. The GPIO interrupt is routed to the
     * core that installed the service, so give an edge already running there a full tick to finish (a delay of 1
     * can end at the next tick interrupt, which may be microseconds away). bm_hal_gpio_off also disables the pin's
     * interrupt. */
    bm_hal_gpio_off(ih->pin);
    vTaskDelay(2);
    gpio_reset_pin((gpio_num_t) ih->pin);
    executor_irq_cancel(ih->job);
    free(ih);
}

const char *bm_i2c_hal_err_name(bm_err_t err)
{
    if (err == BM_ERR_ADDR_IN_USE) {
        return "address already used by another driver on this bus";
    }
    if (err == BM_ERR_BUS_PINS) {
        return "the port or a pin is already used by an I2C bus open with other pins";
    }
    return esp_err_to_name(err);
}

void bm_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}
