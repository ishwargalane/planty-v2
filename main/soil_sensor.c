/*
 * Soil Moisture Sensor Driver Implementation
 */
#include "soil_sensor.h"
#include "sensor_config.h"
#include <ads111x.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char *TAG = "SOIL_SENSOR";
static i2c_dev_t ads_dev;
static bool initialized = false;
static float gain_val;  // Store gain value for voltage calculation

esp_err_t soil_sensor_init(void)
{
    esp_err_t ret;

    if (initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Clear device descriptor (following library example pattern)
    memset(&ads_dev, 0, sizeof(i2c_dev_t));
    
    // Initialize ADS1115 device descriptor
    ret = ads111x_init_desc(&ads_dev, ADS1115_ADDR, I2C_MASTER_NUM, 
                            I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADS1115 descriptor: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "ADS1115 descriptor initialized (addr=0x%02X)", ADS1115_ADDR);

    // Configure gain: ±4.096V range (suitable for 3.3V sensors)
    ret = ads111x_set_gain(&ads_dev, ADS111X_GAIN_4V096);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gain: %s", esp_err_to_name(ret));
        ads111x_free_desc(&ads_dev);
        return ret;
    }
    
    // Store gain value for voltage calculations
    gain_val = ads111x_gain_values[ADS111X_GAIN_4V096];
    ESP_LOGI(TAG, "Gain set to ±4.096V (gain_val=%.3f)", gain_val);

    // Set data rate to 128 samples per second (good balance of speed and accuracy)
    ret = ads111x_set_data_rate(&ads_dev, ADS111X_DATA_RATE_128);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set data rate: %s", esp_err_to_name(ret));
        ads111x_free_desc(&ads_dev);
        return ret;
    }

    // Set mode to single-shot (power efficient for 15-min intervals)
    ret = ads111x_set_mode(&ads_dev, ADS111X_MODE_SINGLE_SHOT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set mode: %s", esp_err_to_name(ret));
        ads111x_free_desc(&ads_dev);
        return ret;
    }

    initialized = true;
    ESP_LOGI(TAG, "ADS1115 initialized successfully (single-shot mode, 128 SPS)");
    
    return ESP_OK;
}

esp_err_t soil_sensor_read_channel(uint8_t channel, int16_t *raw_value, float *voltage)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Sensor not initialized. Call soil_sensor_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (channel >= 4) {
        ESP_LOGE(TAG, "Invalid channel: %d (must be 0-3)", channel);
        return ESP_ERR_INVALID_ARG;
    }

    if (raw_value == NULL) {
        ESP_LOGE(TAG, "NULL pointer for raw_value");
        return ESP_ERR_INVALID_ARG;
    }

    // Map channel to MUX setting (single-ended input vs GND)
    ads111x_mux_t mux_settings[] = {
        ADS111X_MUX_0_GND,  // A0 vs GND
        ADS111X_MUX_1_GND,  // A1 vs GND
        ADS111X_MUX_2_GND,  // A2 vs GND
        ADS111X_MUX_3_GND   // A3 vs GND
    };

    esp_err_t ret;
    
    // Set the input multiplexer
    ret = ads111x_set_input_mux(&ads_dev, mux_settings[channel]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set input mux for channel %d: %s", 
                 channel, esp_err_to_name(ret));
        return ret;
    }

    // Small delay for mux to settle
    vTaskDelay(pdMS_TO_TICKS(10));

    // In single-shot mode, ads111x_get_value() triggers conversion and waits for completion
    ret = ads111x_get_value(&ads_dev, raw_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read from channel %d: %s", 
                 channel, esp_err_to_name(ret));
        return ret;
    }

    // Calculate voltage if requested (following library example formula)
    if (voltage != NULL) {
        *voltage = gain_val / ADS111X_MAX_VALUE * (*raw_value);
        ESP_LOGD(TAG, "Ch%d: raw=%d, voltage=%.4fV", channel, *raw_value, *voltage);
    } else {
        ESP_LOGD(TAG, "Ch%d: raw=%d", channel, *raw_value);
    }
    
    return ESP_OK;
}

float soil_sensor_raw_to_percentage(int16_t raw_value)
{
    /* 
     * Convert ADC value to moisture percentage
     * 
     * Soil moisture sensors work by measuring capacitance/resistance:
     * - Higher ADC value = drier soil (higher voltage, less conductivity)
     * - Lower ADC value = wetter soil (lower voltage, more conductivity)
     * 
     * We map the raw value to 0-100% using calibration values:
     * - MOISTURE_DRY_VALUE → 0%
     * - MOISTURE_WET_VALUE → 100%
     */
    
    float percentage;
    
    if (raw_value >= MOISTURE_DRY_VALUE) {
        percentage = 0.0f;  // Completely dry
    } else if (raw_value <= MOISTURE_WET_VALUE) {
        percentage = 100.0f;  // Completely wet
    } else {
        // Linear interpolation between wet and dry
        percentage = 100.0f * (float)(MOISTURE_DRY_VALUE - raw_value) / 
                     (float)(MOISTURE_DRY_VALUE - MOISTURE_WET_VALUE);
    }
    
    // Clamp to valid range
    if (percentage < 0.0f) percentage = 0.0f;
    if (percentage > 100.0f) percentage = 100.0f;
    
    return percentage;
}

esp_err_t soil_sensor_read_all(float *sensor_values, uint8_t num_sensors)
{
    if (!initialized) {
        ESP_LOGE(TAG, "Sensor not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (sensor_values == NULL) {
        ESP_LOGE(TAG, "NULL pointer for sensor_values");
        return ESP_ERR_INVALID_ARG;
    }

    if (num_sensors > 4 || num_sensors == 0) {
        ESP_LOGE(TAG, "Invalid number of sensors: %d (must be 1-4)", num_sensors);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    int16_t raw_value;
    float voltage;

    ESP_LOGI(TAG, "Reading %d soil moisture sensor(s)...", num_sensors);

    for (uint8_t i = 0; i < num_sensors; i++) {
        ret = soil_sensor_read_channel(i, &raw_value, &voltage);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read sensor %d", i);
            sensor_values[i] = -1.0f;  // Error indicator
            continue;
        }
        
        sensor_values[i] = soil_sensor_raw_to_percentage(raw_value);
        ESP_LOGI(TAG, "Sensor %d: raw=%d, voltage=%.4fV, moisture=%.1f%%", 
                 i, raw_value, voltage, sensor_values[i]);
        
        // Small delay between channel readings for stability
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return ESP_OK;
}

esp_err_t soil_sensor_deinit(void)
{
    if (!initialized) {
        return ESP_OK;
    }

    esp_err_t ret = ads111x_free_desc(&ads_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to free ADS1115 descriptor: %s", esp_err_to_name(ret));
        return ret;
    }

    initialized = false;
    ESP_LOGI(TAG, "ADS1115 deinitialized");
    
    return ESP_OK;
}

void soil_sensor_calibration_mode(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║      SOIL MOISTURE SENSOR CALIBRATION MODE                 ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Instructions:");
    ESP_LOGI(TAG, "  1. Keep sensor probe in DRY AIR");
    ESP_LOGI(TAG, "     → Note the RAW value (this is MOISTURE_DRY_VALUE)");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  2. Submerge sensor probe in WATER");
    ESP_LOGI(TAG, "     → Note the RAW value (this is MOISTURE_WET_VALUE)");
    ESP_LOGI(TAG, "     ⚠️  DO NOT submerge the electronics, only the probe!");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "  3. Update these values in sensor_config.h:");
    ESP_LOGI(TAG, "     #define MOISTURE_DRY_VALUE   <your_dry_value>");
    ESP_LOGI(TAG, "     #define MOISTURE_WET_VALUE   <your_wet_value>");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Reading every 2 seconds...");
    ESP_LOGI(TAG, "══════════════════════════════════════════════════════════════");
    
    int16_t raw_value;
    float voltage;
    
    while (1) {
        ESP_LOGI(TAG, "");
        for (int channel = 0; channel < NUM_SOIL_SENSORS; channel++) {
            if (soil_sensor_read_channel(channel, &raw_value, &voltage) == ESP_OK) {
                ESP_LOGI(TAG, "Channel %d │ RAW: %6d │ Voltage: %.4f V", 
                         channel, raw_value, voltage);
            } else {
                ESP_LOGE(TAG, "Channel %d │ READ ERROR", channel);
            }
        }
        ESP_LOGI(TAG, "──────────────────────────────────────────────────────────────");
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}