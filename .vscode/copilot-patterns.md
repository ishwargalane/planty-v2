# ESP32 Soil Moisture Monitor - Common Code Patterns

This file documents common code patterns specific to this project for GitHub Copilot reference.

## ESP RainMaker Device Creation Patterns

### Creating a Moisture Sensor Device
```c
// Moisture sensors use Temperature Sensor type for 0-100% display
esp_rmaker_device_t *moisture_sensor = esp_rmaker_temp_sensor_device_create(
    "Moisture Sensor 1", 
    NULL,  // friendly_name (optional)
    0.0    // initial value
);

if (moisture_sensor) {
    // Add device to node
    esp_rmaker_node_add_device(node, moisture_sensor);
    
    // Add callback for parameter updates
    esp_rmaker_device_add_cb(moisture_sensor, write_cb, NULL);
    
    // Store device handle for later updates
    g_moisture_sensor_1 = moisture_sensor;
}
```

### Creating a Switch Device
```c
esp_rmaker_device_t *switch_device = esp_rmaker_switch_device_create(
    "Switch",
    NULL,
    DEFAULT_POWER
);

if (switch_device) {
    esp_rmaker_node_add_device(node, switch_device);
    esp_rmaker_device_add_cb(switch_device, write_cb, user_data);
}
```

### Updating RainMaker Parameters
```c
// Update and report to cloud (triggers cloud sync)
esp_rmaker_param_t *param = esp_rmaker_device_get_param_by_type(
    device,
    ESP_RMAKER_PARAM_TEMPERATURE
);

if (param) {
    esp_rmaker_param_update_and_report(param, esp_rmaker_float(moisture_value));
}

// Update and notify locally (no cloud sync)
esp_rmaker_param_update_and_notify(param, esp_rmaker_float(moisture_value));
```

## ADS1115 I2C Sensor Reading Pattern

### Initialize ADS1115 Device
```c
#include <ads111x.h>

static i2c_dev_t ads_dev = {0};

esp_err_t init_ads1115(void) {
    // Initialize I2C device descriptor
    esp_err_t err = ads111x_init_desc(
        &ads_dev,
        ADS111X_ADDR_GND,  // Address (ADDR pin to GND = 0x48)
        I2C_PORT,          // I2C port (usually 0)
        SDA_GPIO,          // SDA pin
        SCL_GPIO           // SCL pin
    );
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADS111x descriptor: %s", esp_err_to_name(err));
        return err;
    }
    
    // Set gain and data rate
    err = ads111x_set_gain(&ads_dev, ADS111X_GAIN_4V096);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gain: %s", esp_err_to_name(err));
        return err;
    }
    
    err = ads111x_set_data_rate(&ads_dev, ADS111X_DATA_RATE_128);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set data rate: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "ADS1115 initialized successfully");
    return ESP_OK;
}
```

### Read Single-Ended Channel
```c
esp_err_t read_moisture_sensor(uint8_t channel, int16_t *raw_value, float *moisture_percent) {
    // Set input multiplexer (AIN0-GND, AIN1-GND, etc.)
    ads111x_mux_t mux;
    switch (channel) {
        case 0: mux = ADS111X_MUX_0_GND; break;
        case 1: mux = ADS111X_MUX_1_GND; break;
        case 2: mux = ADS111X_MUX_2_GND; break;
        case 3: mux = ADS111X_MUX_3_GND; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t err = ads111x_set_input_mux(&ads_dev, mux);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set mux: %s", esp_err_to_name(err));
        return err;
    }
    
    // Start conversion and wait
    err = ads111x_start_conversion(&ads_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start conversion: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for conversion (approximately 8ms at 128 SPS)
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Check if conversion is ready
    bool busy;
    err = ads111x_is_busy(&ads_dev, &busy);
    if (err != ESP_OK || busy) {
        ESP_LOGE(TAG, "Conversion not ready");
        return ESP_ERR_TIMEOUT;
    }
    
    // Read raw ADC value
    err = ads111x_get_value(&ads_dev, raw_value);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read value: %s", esp_err_to_name(err));
        return err;
    }
    
    // Calculate moisture percentage
    *moisture_percent = calculate_moisture_percentage(*raw_value);
    
    ESP_LOGI(TAG, "Channel %d: Raw=%d, Moisture=%.1f%%", channel, *raw_value, *moisture_percent);
    return ESP_OK;
}
```

### Moisture Percentage Calculation
```c
float calculate_moisture_percentage(int16_t raw_value) {
    // Linear interpolation between DRY_VALUE and WET_VALUE
    // DRY = 0%, WET = 100%
    
    if (raw_value >= DRY_VALUE) {
        return 0.0;  // Completely dry
    }
    if (raw_value <= WET_VALUE) {
        return 100.0;  // Completely wet
    }
    
    // Linear mapping: higher ADC value = drier soil
    float moisture = 100.0 * (float)(DRY_VALUE - raw_value) / (float)(DRY_VALUE - WET_VALUE);
    
    return moisture;
}
```

## FreeRTOS Task Patterns

### Sensor Reading Task
```c
static const char *TAG = "sensor_task";

void sensor_reading_task(void *pvParameters) {
    const TickType_t reading_interval = pdMS_TO_TICKS(60000);  // 60 seconds
    
    ESP_LOGI(TAG, "Sensor reading task started");
    
    while (1) {
        // Read all sensors
        for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
            int16_t raw_value;
            float moisture;
            
            esp_err_t err = read_moisture_sensor(i, &raw_value, &moisture);
            if (err == ESP_OK) {
                // Update RainMaker parameter
                update_sensor_value(i, moisture);
            } else {
                ESP_LOGE(TAG, "Failed to read sensor %d: %s", i, esp_err_to_name(err));
            }
            
            // Small delay between sensor reads
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // Wait for next reading cycle
        vTaskDelay(reading_interval);
    }
}
```

### Creating a Task
```c
BaseType_t result = xTaskCreate(
    sensor_reading_task,    // Task function
    "sensor_task",          // Task name (for debugging)
    4096,                   // Stack size (bytes)
    NULL,                   // Parameters
    5,                      // Priority
    &sensor_task_handle     // Task handle (optional)
);

if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create sensor task");
    return ESP_FAIL;
}
```

## Error Handling Patterns

### Standard ESP-IDF Error Check
```c
esp_err_t err = some_esp_function();
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Function failed: %s", esp_err_to_name(err));
    return err;
}
```

### Error Check with Cleanup
```c
esp_err_t err = initialize_peripheral();
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(err));
    cleanup_resources();
    return err;
}
```

### Using ESP_ERROR_CHECK (only for critical errors)
```c
// This will abort the program if error occurs
ESP_ERROR_CHECK(nvs_flash_init());
```

### Using ESP_GOTO_ON_ERROR
```c
esp_err_t ret = ESP_OK;
i2c_dev_t *dev = NULL;

ESP_GOTO_ON_ERROR(i2c_init(), cleanup, TAG, "I2C init failed");
ESP_GOTO_ON_ERROR(sensor_init(), cleanup, TAG, "Sensor init failed");

return ESP_OK;

cleanup:
    if (dev) {
        i2c_dev_delete(dev);
    }
    return ret;
```

## Logging Patterns

### Standard Logging
```c
static const char *TAG = "module_name";

ESP_LOGI(TAG, "Informational message: value=%d", value);
ESP_LOGW(TAG, "Warning: threshold exceeded");
ESP_LOGE(TAG, "Error occurred: %s", esp_err_to_name(err));
ESP_LOGD(TAG, "Debug info: sensor=%d, raw=%d", sensor_id, raw_value);
```

### Conditional Debug Logging
```c
#if CONFIG_LOG_DEFAULT_LEVEL >= ESP_LOG_DEBUG
    ESP_LOGD(TAG, "Verbose debug information");
#endif
```

## I2C Scanner Pattern

### Scanning I2C Bus
```c
void i2c_scanner_scan(i2c_port_t port) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02x", addr);
        }
    }
    
    ESP_LOGI(TAG, "I2C scan complete");
}
```

## NVS (Non-Volatile Storage) Pattern

### Read from NVS
```c
nvs_handle_t nvs_handle;
esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
if (err == ESP_OK) {
    int32_t value = 0;
    err = nvs_get_i32(nvs_handle, "key_name", &value);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Read value: %ld", value);
    }
    nvs_close(nvs_handle);
}
```

### Write to NVS
```c
nvs_handle_t nvs_handle;
esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
if (err == ESP_OK) {
    err = nvs_set_i32(nvs_handle, "key_name", value);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);  // Don't forget to commit!
    }
    nvs_close(nvs_handle);
}
```

## Button Handling Pattern

### Button Event Handler
```c
static void button_press_down_cb(void *arg, void *data) {
    ESP_LOGI(TAG, "Button pressed");
    // Toggle state
    app_driver_toggle_state();
}

static void button_press_long_cb(void *arg, void *data) {
    ESP_LOGI(TAG, "Button long press - resetting to factory");
    nvs_flash_erase();
    esp_restart();
}
```

## LED Indicator Pattern

### Set LED Color
```c
// Green for ON state
app_indicator_set(0, 255, 0);  // R, G, B

// Red for error state
app_indicator_set(255, 0, 0);

// Blue for connecting
app_indicator_set(0, 0, 255);

// Off
app_indicator_set(0, 0, 0);
```

## Calibration Pattern

### Interactive Calibration Routine
```c
void calibrate_sensor(uint8_t sensor_id) {
    ESP_LOGI(TAG, "Starting calibration for sensor %d", sensor_id);
    ESP_LOGI(TAG, "Place sensor in DRY soil and press button...");
    
    // Wait for button press
    wait_for_button();
    
    int16_t dry_value;
    read_sensor_raw(sensor_id, &dry_value);
    ESP_LOGI(TAG, "DRY value: %d", dry_value);
    
    ESP_LOGI(TAG, "Place sensor in WET soil and press button...");
    wait_for_button();
    
    int16_t wet_value;
    read_sensor_raw(sensor_id, &wet_value);
    ESP_LOGI(TAG, "WET value: %d", wet_value);
    
    // Store calibration values in NVS
    save_calibration(sensor_id, dry_value, wet_value);
}
```

---

**Note**: All patterns follow ESP-IDF conventions and use proper error handling, logging, and resource management.
