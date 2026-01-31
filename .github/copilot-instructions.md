# Planty-v2 Codebase Guide for AI Agents

**Project**: ESP32-based plant moisture monitoring system with cloud connectivity via ESP RainMaker

## Required External References (consult before making RainMaker/ESP-IDF changes)

- ESP RainMaker repository: https://github.com/espressif/esp-rainmaker — use for device APIs, examples, and agent internals.
- ESP-IDF repository: https://github.com/espressif/esp-idf - core SDK, FreeRTOS, peripheral drivers.
- RainMaker firmware code overview: https://docs.rainmaker.espressif.com/docs/dev/firmware/fw_usage_guides/code-overview/ — patterns and expected app flow.
- ESP-IDF API index: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/index.html — peripheral, system, and RTOS APIs.


Agents: always consult these links when changing RainMaker device models or ESP-IDF APIs; cite which link(s) you used in PR descriptions or commit messages.

## Architecture Overview

**Planty** is an embedded IoT application featuring:
- **Cloud integration**: ESP RainMaker for remote device control and monitoring via phone app
- **Hardware abstraction**: Modular drivers for sensors (soil moisture via ADS1115 ADC), buttons, and LED indicators
- **Multi-device model**: Each monitoring point (switch, light, fan, temperature, soil sensors) is a distinct RainMaker device

### Key Files & Responsibilities

| File | Purpose |
|------|---------|
| [main/app_main.c](main/app_main.c) | RainMaker initialization, device creation, command callbacks (write_cb). Soil sensor device array init in `init_soil_sensor_devices()` |
| [main/app_driver.c](main/app_driver.c) | Hardware control: button handling, LED RGB control, FreeRTOS timers for periodic sensor updates (app_sensor_update, app_soil_sensor_update) |
| [main/soil_sensor.h](main/soil_sensor.h) | ADS1115 ADC driver interface: `soil_sensor_init()`, `soil_sensor_read_all()`, raw-to-percentage conversion |
| [main/soil_sensor.c](main/soil_sensor.c) | ADS1115 implementation using esp-idf-lib; single-shot mode ±4.096V range |
| [main/sensor_config.h](main/sensor_config.h) | I2C pins (SDA=21, SCL=22), calibration values (MOISTURE_DRY_VALUE=17500, MOISTURE_WET_VALUE=7600), NUM_SOIL_SENSORS=4 |
| [main/app_priv.h](main/app_priv.h) | Global device pointers and defaults (e.g., DEFAULT_SWITCH_POWER, REPORTING_PERIOD=60s) |
| [main/Kconfig.projbuild](main/Kconfig.projbuild) | menuconfig options for BOOT button, output GPIO, LED type/GPIO |

## Critical Workflows

### Build & Flash
```bash
idf.py build              # Compile (default target: esp32)
idf.py -p /dev/ttyUSB0 flash monitor  # Flash and monitor serial output
idf.py set-target esp32s3 # Switch target (updates sdkconfig)
```
For multiple board targets, use `sdkconfig.defaults.esp32`, `sdkconfig.defaults.esp32c3`, etc. stored in repo root.

### Debugging Hardware
- **I2C Scanner**: Run `i2c_scanner()` in app_driver initialization to verify ADS1115 presence (default address 0x48)
- **Calibration**: Call `soil_sensor_calibration_mode()` from app_driver to continuously stream raw ADC values; find dry/wet thresholds and update [sensor_config.h](main/sensor_config.h)
- **Button/LED**: Configure via Kconfig (CONFIG_EXAMPLE_BOARD_BUTTON_GPIO, CONFIG_LED_TYPE_RGB/WS2812, CONFIG_WS2812_LED_GPIO)

### Adding Sensors
1. Add channel to [sensor_config.h](main/sensor_config.h): `#define NUM_SOIL_SENSORS 5`
2. Extend array in [app_main.c](main/app_main.c) line ~78: `soil_sensor_devices[NUM_SOIL_SENSORS]`
3. Loop creates device in `init_soil_sensor_devices()` using `esp_rmaker_temp_sensor_device_create()`
4. Update callback in `app_soil_sensor_update()` timer to report all values

## Data Flows & Integration Points

### Sensor → RainMaker → App
1. **Periodic reads** (60s): `app_soil_sensor_update()` timer calls `soil_sensor_read_all()`
2. **Raw → Calibrated**: `soil_sensor_raw_to_percentage()` maps ADC values using dry/wet thresholds
3. **RainMaker update**: `esp_rmaker_param_update_and_report()` syncs to cloud; triggers phone app notification

### Command from App → Device
1. Phone app sends command → RainMaker cloud
2. ESP32 receives via `write_cb()` (callback registered on each device)
3. `write_cb()` matches device name + parameter name (e.g., "Switch" + "power") and calls driver (e.g., `app_driver_set_state()`)
4. LED/GPIO state updates; parameter value confirmed via `esp_rmaker_param_update()`

## Project-Specific Patterns

### RainMaker Device Model
- Each device has **parameters** (e.g., "power", "brightness", "speed", "temperature")
- Standard param names in `<esp_rmaker_standard_params.h>`: ESP_RMAKER_DEF_POWER_NAME, ESP_RMAKER_DEF_BRIGHTNESS_NAME
- Write callback must check **both** device name AND parameter name to route commands correctly

### LED RGB Control
- Percentage values (0–100) converted to 0–255 range: `r_255 = (red * 255) / 100`
- Use `led_indicator_set_rgb(handle, SET_IRGB(MAX_INDEX, r, g, b))`; see [app_driver.c](main/app_driver.c) line ~65

### FreeRTOS Timers
- Sensor updates use `xTimerCreate()` + `xTimerStart()` in `app_driver_init()`
- Timer callbacks read hardware and call RainMaker param update; avoid blocking calls inside callbacks

### I2C & ADS1115
- Single i2c bus (I2C_NUM_0) with SDA/SCL defined in [sensor_config.h](main/sensor_config.h)
- ADS1115 uses 16-bit reads; esp-idf-lib abstracts device handle (`ads111x_t`)
- Must call `i2cdev_init()` before `soil_sensor_init()`; see [app_driver.c](main/app_driver.c) line ~120

## Build Configuration & Dependencies

**Components** (via idf_component.yml):
- `espressif/esp_rainmaker` ≥1.0 (cloud/provisioning)
- `espressif/button` ^4.1.4 (button debounce/events)
- `espressif/led_indicator` ^2.0.0 (RGB/WS2812 abstraction)
- `esp-idf-lib/ads111x` (ADS1115 ADC driver)
- Custom overrides for rmaker_app_reset, rmaker_app_network, rmaker_app_insights

**Partition table**: See [partitions.csv](partitions.csv) and [partitions_4mb_optimised.csv](partitions_4mb_optimised.csv); firmware split between app and OTA slots

## Common Pitfalls

1. **Uninitialized soil sensor**: If `soil_sensor_read_all()` returns errors, verify I2C init order and ADS1115 address in menuconfig
2. **RainMaker callback typos**: Parameter name strings (e.g., ESP_RMAKER_DEF_POWER_NAME) must match exactly; check [esp_rmaker_standard_params.h](https://github.com/espressif/esp-rainmaker/blob/main/components/esp_rainmaker/include/esp_rmaker_standard_params.h) reference
3. **Calibration values**: MOISTURE_DRY_VALUE and MOISTURE_WET_VALUE are per-sensor hardware-specific; dry must be > wet (inverse ADC logic)
4. **Device naming**: In `write_cb()`, strcmp device names must match creation names in `init_soil_sensor_devices()` (e.g., "Soil Moisture 1")

---

**Last Updated**: 2026-01-31 | **Target ESP-IDF**: 5.5.2
