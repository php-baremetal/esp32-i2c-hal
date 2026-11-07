# Changelog

## 1.1.1

- Added: bm_i2c_hal_irq_start(), _after(), _pause(), _stop() over the executor's on-demand jobs; the package now provides php-baremetal/i2c-hal 1.2.0.
- Requires php-baremetal/esp32-executor ^1.1.1, php-baremetal/events-hal ^1.4 and the IDF component esp_driver_gpio: the INT pin is attached through the events HAL (bm_hal_gpio_on_falling/off), which installs the GPIO ISR service once, so a firmware with ext-display's TE no longer logs "GPIO isr service already installed".

## 1.1.0

- Added: bm_i2c_hal_poll_pause() over executor_poll_pause(); the package now provides php-baremetal/i2c-hal 1.1.0.
