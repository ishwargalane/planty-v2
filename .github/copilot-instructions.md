# ESP32 Soil Moisture Monitor - GitHub Copilot Instructions

## Project Context

This is an **ESP32-based IoT soil moisture monitoring system** ("planty-v2") built on ESP-IDF with ESP RainMaker cloud integration. The project monitors up to 4 capacitive soil moisture sensors using an ADS1115 16-bit ADC over I2C and reports moisture levels to the ESP RainMaker cloud platform for remote monitoring via mobile apps.

### Core Technologies
- **ESP-IDF v5.0+** - Espressif IoT Development Framework
- **ESP RainMaker** - Cloud platform for device management and phone apps
- **FreeRTOS** - Real-time operating system (built into ESP-IDF)
- **Hardware**: ESP32, ADS1115 ADC (I2C 0x48), capacitive soil moisture sensors, WS2812 RGB LED

### Target Hardware
- Primary: ESP32 (configurable for ESP32-S2, C3, C6, H2, etc.)
- I2C Pins: SDA=GPIO21, SCL=GPIO22 (configurable in sensor_config.h)
- LED: WS2812 RGB on GPIO18
- Button: BOOT button for user input

## Essential Documentation References

**Always consult these resources when implementing features:**

### ESP RainMaker Documentation
- **Main Docs**: https://docs.rainmaker.espressif.com/
- **Technical Overview**: https://docs.rainmaker.espressif.com/docs/product_overview/technical_overview/introduction
- **API Reference**: https://docs.rainmaker.espressif.com/docs/apis/
- **Device API**: https://docs.rainmaker.espressif.com/docs/apis/device-api
- **Node API**: https://docs.rainmaker.espressif.com/docs/apis/node-api
- **Parameter API**: https://docs.rainmaker.espressif.com/docs/apis/parameter-api
- **Firmware Guide**: https://evaluation.rainmaker.espressif.com/firmware-developer-guide

### ESP-IDF Documentation
- **ESP-IDF API Reference**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/
- **I2C Driver**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2c.html
- **NVS Storage**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/storage/nvs_flash.html
- **WiFi API**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html
- **FreeRTOS**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html
- **Logging**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/log.html

### Component Libraries
- **esp-idf-lib** (ADS111x driver): https://github.com/UncleRus/esp-idf-lib
- **ADS111x Component**: https://github.com/UncleRus/esp-idf-lib/tree/master/components/ads111x
- **ESP RainMaker GitHub**: https://github.com/espressif/esp-rainmaker
- **ESP RainMaker Examples**: https://github.com/espressif/esp-rainmaker/tree/master/examples

### Community Resources
- **ESP32 Forum**: https://esp32.com/
- **ESP-IDF GitHub Issues**: https://github.com/espressif/esp-idf/issues
- **ESP RainMaker Forum**: https://rainmaker.espressif.com/forum/

## Code Style & Conventions

### General Coding Standards
- Follow **ESP-IDF coding standards** and conventions
- Use descriptive variable names (snake_case for variables/functions)
- Add Doxygen-style comments for public functions
- Keep functions focused and modular

### Logging
- **Always use ESP-IDF logging macros**: ESP_LOGI(), ESP_LOGE(), ESP_LOGW(), ESP_LOGD()
- Define log tags at file level: `static const char *TAG = "module_name";`
- Example:
  ```c
  static const char *TAG = "soil_sensor";
  ESP_LOGI(TAG, "Moisture sensor reading: %.1f%%", moisture);
  ESP_LOGE(TAG, "Failed to read sensor: %s", esp_err_to_name(err));
  ```

### Error Handling
- **Always check `esp_err_t` return values**
- Use ESP-IDF error checking macros: `ESP_ERROR_CHECK()`, `ESP_GOTO_ON_ERROR()`
- Example:
  ```c
  esp_err_t err = ads111x_init_desc(&dev, ADS111X_ADDR_GND, I2C_PORT, SDA_GPIO, SCL_GPIO);
  if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init ADS111x: %s", esp_err_to_name(err));
      return err;
  }
  ```

### FreeRTOS Conventions
- Use `pdMS_TO_TICKS()` for time conversions in FreeRTOS functions
- Use `vTaskDelay(pdMS_TO_TICKS(1000))` instead of raw tick values
- Always check task creation return values

### I2C Operations
- Follow **esp-idf-lib** conventions for I2C device descriptors
- Always initialize device descriptors with proper error checking
- Lock I2C bus when needed for multi-sensor operations

### ESP RainMaker Patterns
- Create devices with appropriate device types: `esp_rmaker_switch_device_create()`, `esp_rmaker_temp_sensor_device_create()`
- Add devices to node: `esp_rmaker_node_add_device(node, device)`
- Update parameters: `esp_rmaker_param_update_and_report()` or `esp_rmaker_param_update_and_notify()`
- Always check for NULL pointers from RainMaker create functions

## Project Structure & Key Files

### Main Application Files
- **main/app_main.c** - Entry point, RainMaker node initialization, device creation
- **main/soil_sensor.c/.h** - ADS1115 driver, sensor reading, calibration routines
- **main/app_driver.c** - Hardware abstraction layer (LED, button)
- **main/sensor_config.h** - Hardware configuration (I2C pins, calibration values, sensor count)
- **main/app_priv.h** - Private application headers and definitions
- **main/i2c_scanner.c/.h** - I2C bus debugging utility

### Build Configuration
- **CMakeLists.txt** - Top-level CMake project configuration
- **main/CMakeLists.txt** - Component CMake configuration
- **main/idf_component.yml** - Component dependencies (esp-idf-lib, button, led_indicator)
- **sdkconfig.defaults*** - Target-specific SDK configurations
- **partitions*.csv** - Flash memory partition layouts

## Hardware Configuration

### Current Settings (sensor_config.h)
- **NUM_SOIL_SENSORS**: 4 (maximum sensors supported)
- **I2C Configuration**:
  - SDA_GPIO: 21
  - SCL_GPIO: 22
  - I2C_FREQ_HZ: 100000 (100 kHz)
- **Calibration Values** (ADC raw values):
  - DRY_VALUE: 17500
  - WET_VALUE: 7600
- **Reading Interval**: 60 seconds (configurable)
- **ADS1115 Address**: 0x48 (ADDR pin to GND)

### Device Mapping
- Moisture sensors use **"Temperature Sensor"** device type for RainMaker display (shows 0-100%)
- Each sensor is a separate RainMaker device: "Moisture Sensor 1" through "Moisture Sensor 4"

## Common Implementation Patterns

### Adding a New Sensor Device
```c
// Create device
esp_rmaker_device_t *sensor = esp_rmaker_temp_sensor_device_create("Moisture Sensor 1", NULL, 0.0);
if (sensor) {
    esp_rmaker_node_add_device(node, sensor);
    esp_rmaker_device_add_cb(sensor, sensor_callback_fn, NULL);
}
```

### Reading ADS1115 Sensor
```c
esp_err_t err = ads111x_get_value(&dev, &raw_value);
if (err == ESP_OK) {
    float moisture = calculate_moisture_percentage(raw_value);
    ESP_LOGI(TAG, "Sensor %d: %.1f%%", sensor_num, moisture);
}
```

### Updating RainMaker Parameter
```c
esp_rmaker_param_update_and_report(
    esp_rmaker_device_get_param_by_type(device, ESP_RMAKER_PARAM_TEMPERATURE),
    esp_rmaker_float(moisture_value)
);
```

### FreeRTOS Task Pattern
```c
void sensor_task(void *pvParameters) {
    while (1) {
        // Read sensors
        read_all_sensors();
        
        // Wait for next reading
        vTaskDelay(pdMS_TO_TICKS(READING_INTERVAL_MS));
    }
}
```

## Build System

### Standard ESP-IDF Commands
- **Build**: `idf.py build`
- **Flash**: `idf.py flash`
- **Monitor**: `idf.py monitor`
- **Clean**: `idf.py fullclean`
- **Menuconfig**: `idf.py menuconfig`
- **Set Target**: `idf.py set-target esp32`

### Component Dependencies
Managed in `main/idf_component.yml`:
- `espressif/esp_rainmaker` - RainMaker cloud integration
- `esp-idf-lib/ads111x` - ADS1115 ADC driver
- `espressif/button` - Button handling
- `espressif/led_indicator` - LED control

## Development Environment

### Dev Container
- Full ESP-IDF toolchain pre-installed
- SSH server enabled for external terminal access
- USB device access configured for flashing
- All required tools: git, python, cmake, ninja, etc.

### VSCode Integration
- ESP-IDF extension for building, flashing, monitoring
- Clangd for IntelliSense
- Debugger configurations for ESP32

## Important Project-Specific Notes

### Current Development Phase
- Project is in **calibration/development phase**
- Sensor calibration values may need adjustment based on soil type
- Demo devices (Switch, Lightbulb, Fan) from base example are still present

### Known Configuration Points
- Moisture sensors show as "Temperature Sensor" type in RainMaker (convention for 0-100% display)
- Calibration: DRY_VALUE and WET_VALUE in sensor_config.h must be tuned per sensor
- I2C bus shared between all sensors on ADS1115 (uses multiplexing)
- RGB LED state indicates switch/device status (green = on)

### Reset to Factory
- Press and hold BOOT button for >3 seconds to reset to factory defaults
- Requires re-provisioning through RainMaker app

## Code Generation Guidelines

When generating or modifying code:

1. **Always reference ESP-IDF and RainMaker documentation** for API usage
2. **Check return values** for all ESP-IDF functions that return `esp_err_t`
3. **Use appropriate log levels** (INFO for normal operation, ERROR for failures, DEBUG for verbose)
4. **Follow existing code patterns** in app_main.c and soil_sensor.c
5. **Respect hardware configuration** defined in sensor_config.h
6. **Test I2C operations** with proper error handling
7. **Update RainMaker parameters** when sensor values change
8. **Consider power management** for battery-operated scenarios
9. **Document calibration procedures** for new sensor types
10. **Maintain compatibility** with ESP RainMaker mobile apps

## Security Considerations

- Never hardcode WiFi credentials (use provisioning)
- Use secure MQTT connections for RainMaker
- Validate all user input from RainMaker callbacks
- Use NVS encryption for sensitive data storage
- Follow ESP-IDF security best practices

## Testing & Debugging

- Use `i2c_scanner` utility to verify I2C devices on bus
- Monitor serial output with `idf.py monitor`
- Check RainMaker node status in mobile app
- Verify sensor readings against known moisture levels
- Use ESP_LOG_DEBUG for detailed sensor readings during calibration

---

**Remember**: When implementing features, always prioritize referencing the official ESP RainMaker and ESP-IDF documentation links above to ensure accurate, up-to-date, and best-practice implementations.
