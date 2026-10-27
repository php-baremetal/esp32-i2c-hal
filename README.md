# esp32-i2c-hal

The ESP32 side of [ext-i2c](package:php-baremetal/ext-i2c): it implements the neutral I2C HAL
(`bm_i2c_hal.h`, shipped by ext-i2c) over the ESP-IDF I2C master driver, and runs `Bus::CORE1` buses and
background samplers on [esp32-executor](package:php-baremetal/esp32-executor). It adds no PHP API of its own.

| Property | Value |
| --- | --- |
| Type | `bm-hal` |
| Platform | `esp32` (`extra.baremetal.platform`) |
| Provides | `php-baremetal/i2c-hal` |
| Requires | `php-baremetal/esp32-executor` |
| ESP-IDF components (`idf-requires`) | `esp_driver_i2c` |

## Install

You do not require it. ext-i2c requires the virtual `php-baremetal/i2c-hal`, and conductor adds this package
when the project's SDK is on the `esp32` platform (`"sdk": "esp32@^2.0"`). Require the extension, or a driver:

```bash
composer require php-baremetal/ext-i2c
```

The PHP API, limits and errors are documented in
[ext-i2c](package:php-baremetal/ext-i2c#docs/overview.md).

## What it does on ESP32

- **Opening a bus.** `i2c_new_master_bus()` on the requested port, or the first free one when the port is
  automatic, with a glitch filter of 7 clock cycles and the internal pull-ups on SDA and SCL. The bus is reset
  right after it opens.
- **Devices.** Each device is added to its bus with 7-bit addressing and its own SCL clock.
- **Transactions.** Every read, write and register access has a 1000 ms timeout. A transaction that ends in
  `ESP_ERR_TIMEOUT` or `ESP_ERR_INVALID_STATE` resets the bus and is retried once. A register write longer than
  32 bytes allocates a temporary buffer.
- **Probe and scan.** A probe waits 50 ms and is never retried. A scan probes each address in turn and gives up
  after 8 consecutive timeouts, resetting the bus, which ext-i2c reports as a stuck bus.
- **Ownership.** A `Bus::SYNC` bus is used from the PHP task under a FreeRTOS mutex. A `Bus::CORE1` bus is
  opened and used only on the executor task: calls from PHP are queued to it (waiting at most 1000 ms for a
  free queue slot) and run inline when already on that task, so its interrupt and all its traffic stay on one
  task. The executor is pinned to core 0.
- **Background sampling.** `poll()` starts an executor poller that calls the driver's sample function at the
  requested rate; the sample function takes the bus lock (or runs inline on a `CORE1` bus) itself.
- **Errors.** ext-i2c messages end with the `esp_err_t` name (`ESP_ERR_TIMEOUT`, `ESP_ERR_NOT_FOUND`, …) or with
  one of two php-baremetal texts: `address already used by another driver on this bus` and
  `the port or a pin is already used by an I2C bus open with other pins`.

It also provides `bm_delay_ms()`, the delay driver code uses during chip initialisation (`vTaskDelay`).

## License

MIT.
