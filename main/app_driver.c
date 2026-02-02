/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sdkconfig.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <time.h>
#include <math.h>

#include <i2cdev.h>
#include "soil_sensor.h"
#include <dht.h>
#include "sensor_config.h"

#include <iot_button.h>
#include <button_gpio.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>

#include <i2cdev.h>
#include "soil_sensor.h"
#include <dht.h>
#include "sensor_config.h"

#include <iot_button.h>
#include <button_gpio.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>

#include <app_reset.h>



#include "app_priv.h"

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0
/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    CONFIG_EXAMPLE_OUTPUT_GPIO

/* These values correspoind to H,S,V = 120,100,10 */
#define DEFAULT_RED     0
#define DEFAULT_GREEN   25
#define DEFAULT_BLUE    0

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

/* Add this line right after all the includes */
static const char *TAG = "app_driver";

static bool g_power_state = DEFAULT_SWITCH_POWER;



/* Soil Sensor handle */
static TimerHandle_t soil_sensor_timer;

/* Auto-off interval for pump (in seconds) */
static uint32_t switch_off_interval = 10;  // Default 10 seconds

/* Moisture threshold for auto-watering (in percent) */
static uint32_t moisture_threshold = 30;  // Default 30%

/* Last auto-activation timestamp (for cooldown) */
static time_t last_auto_activation_time = 0;

/* Cooldown period between auto-activations (1 hour) */
#define AUTO_ACTIVATION_COOLDOWN_SEC (60 * 60)  // 1 hour

/* Automatic mode flag */
static bool g_auto_mode = true;  // Default true (Automatic enabled)

/* Cached parameter pointers for efficient updates */
static esp_rmaker_param_t *avg_moisture_param = NULL;
static esp_rmaker_param_t *sensor_params[NUM_SOIL_SENSORS] = {NULL};
static esp_rmaker_param_t *temp_param = NULL;
static esp_rmaker_param_t *humidity_param = NULL;



/* Temperature simulation removed; real sensor (DHT22) will be integrated later */

static void app_soil_sensor_update(TimerHandle_t handle)
{
    float sensor_values[NUM_SOIL_SENSORS];
    
    /* Read all sensors */
    esp_err_t ret = soil_sensor_read_all(sensor_values, NUM_SOIL_SENSORS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read soil sensors");
        return;
    }

    /* Calculate average from all sensors with smart filtering */
    float sum_non_zero = 0.0f;
    int count_non_zero = 0;
    int count_zeros = 0;
    
    for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
        if (sensor_values[i] != -1.0f) {
            if (sensor_values[i] > 0.0f) {
                sum_non_zero += sensor_values[i];
                count_non_zero++;
            } else {
                count_zeros++;
            }
        }
    }
    
    /* Logic:
     * 1. If we have non-zero values, use THEIR average (ignore 0s as potential disconnects)
     * 2. If NO non-zero values but we have zeros, then average is 0.0 (fully dry)
     * 3. If neither, duplicate/invalid
     */
    float average = 0.0f;
    int valid_count = 0;
    
    if (count_non_zero > 0) {
        average = sum_non_zero / count_non_zero;
        valid_count = count_non_zero; // Report count of sensors used for calculation
    } else if (count_zeros > 0) {
        average = 0.0f;
        valid_count = count_zeros;
    } else {
        /* No valid readings */
        valid_count = 0;
    }
    
    /* Update average moisture parameter */
    if (avg_moisture_param) {
        /* Round to 2 decimal places */
        average = roundf(average * 100.0f) / 100.0f;
        esp_rmaker_param_update_and_report(avg_moisture_param, esp_rmaker_float(average));
        ESP_LOGI(TAG, "✓ Average Moisture: %.2f%% (%d/%d sensors)", 
                 average, valid_count, NUM_SOIL_SENSORS);
    }
    
    /* Update individual sensor parameters */
    for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
        if (sensor_values[i] != -1.0f && sensor_params[i]) {
            /* Round to 2 decimal places */
            float rounded = roundf(sensor_values[i] * 100.0f) / 100.0f;
            esp_rmaker_param_update_and_report(sensor_params[i], esp_rmaker_float(rounded));
            ESP_LOGI(TAG, "✓ Sensor %d: %.2f%%", i + 1, rounded);
        }
    }
    
    /* Auto-watering logic: check if moisture below threshold */
    /* Auto-watering logic: check if moisture below or equal to threshold */
    if (valid_count > 0 && average <= moisture_threshold) {
        /* Check cooldown period */
        time_t current_time = time(NULL);
        time_t time_since_last_auto = current_time - last_auto_activation_time;
        
        if (time_since_last_auto >= AUTO_ACTIVATION_COOLDOWN_SEC) {
            /* Check if Automatic mode is enabled */
            if (g_auto_mode) {
                /* Activate pump automatically */
                if (!app_driver_get_state()) {
                    ESP_LOGI(TAG, "🌱 Auto-watering triggered: moisture %.1f%% <= threshold %lu%%", 
                             average, moisture_threshold);
                    app_driver_set_state(true);
                    esp_rmaker_param_update_and_report(
                        esp_rmaker_device_get_param_by_name(pump_device, ESP_RMAKER_DEF_POWER_NAME),
                        esp_rmaker_bool(true));
                    last_auto_activation_time = current_time;
                } else {
                     ESP_LOGI(TAG, "Pump already ON, skipping auto-watering");
                }
            } else {
                /* Auto mode DISABLED: Send Alert Notification */
                ESP_LOGW(TAG, "⚠️ Moisture low (%.1f%%) but Automatic mode is OFF. Sending alert.", average);
                
                char alert_msg[100];
                snprintf(alert_msg, sizeof(alert_msg), "Moisture low (%.1f%%), but Auto-Watering is OFF.", average);
                esp_rmaker_raise_alert(alert_msg);
                
                /* Update last activation time so we don't spam alerts every minute */
                last_auto_activation_time = current_time;
            }
        } else {
            ESP_LOGI(TAG, "⏳ Auto-watering on cooldown: %ld seconds remaining", 
                     (long)(AUTO_ACTIVATION_COOLDOWN_SEC - time_since_last_auto));
        }
    }
}

void app_driver_set_switch_off_interval(uint32_t interval_seconds)
{
    switch_off_interval = interval_seconds;
    ESP_LOGI(TAG, "Switch off interval updated to %lu seconds", interval_seconds);
}

uint32_t app_driver_get_switch_off_interval(void)
{
    return switch_off_interval;
}

void app_driver_set_moisture_threshold(uint32_t threshold_percent)
{
    moisture_threshold = threshold_percent;
    ESP_LOGI(TAG, "Moisture threshold updated to %lu%%", threshold_percent);
}

uint32_t app_driver_get_moisture_threshold(void)
{
    return moisture_threshold;
}

void app_driver_set_auto_mode(bool auto_mode)
{
    g_auto_mode = auto_mode;
    ESP_LOGI(TAG, "Automatic mode updated to %s", auto_mode ? "ON" : "OFF");
}

bool app_driver_get_auto_mode(void)
{
    return g_auto_mode;
}

/* 
This is to switch off the water supply after a set interval. The interval is configurable from the app.
This is to avoid the water supply being on for a long time.
*/
void turnOffSwitchAfterSetInterval(void *pvParameters)
{
    while (1)
    {
        if (app_driver_get_state())
        {   
            /* Use configurable interval instead of hardcoded value */
            vTaskDelay((switch_off_interval * 1000) / portTICK_PERIOD_MS);    
            app_driver_set_state(false);
            esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_name(pump_device, ESP_RMAKER_DEF_POWER_NAME),
                esp_rmaker_bool(false));
            ESP_LOGI(TAG, "Pump turned off after set interval of %lu seconds", switch_off_interval);
        }
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
esp_err_t app_soil_sensor_init(void)
{
    esp_err_t ret;
    
    /* Initialize the soil moisture sensor hardware */
    ret = soil_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize soil sensor hardware");
        return ret;
    }
    
    /* Cache parameter pointers for efficient updates (avoid repeated lookups) */
    if (soil_monitor_device) {
        avg_moisture_param = esp_rmaker_device_get_param_by_name(
            soil_monitor_device, 
            PARAM_AVERAGE_MOISTURE
        );
        
        const char *sensor_names[] = {PARAM_SENSOR_1, PARAM_SENSOR_2, PARAM_SENSOR_3, PARAM_SENSOR_4};
        
        for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
            sensor_params[i] = esp_rmaker_device_get_param_by_name(
                soil_monitor_device,
                sensor_names[i]
            );
        }
        
        ESP_LOGI(TAG, "Parameter pointers cached successfully");
    } else {
        ESP_LOGW(TAG, "Soil monitor device not initialized - cannot cache parameters");
    }
    
    /* Create timer for periodic sensor reading (1 minute) */
    soil_sensor_timer = xTimerCreate(
        "soil_sensor_update_tm",
        (SOIL_SENSOR_READ_INTERVAL_SEC * 1000) / portTICK_PERIOD_MS,
        pdTRUE,                         // Auto-reload
        NULL,                           // Timer ID
        app_soil_sensor_update          // Callback function
    );
    
    if (soil_sensor_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create soil sensor timer");
        return ESP_FAIL;
    }
    
    /* Start the timer - first reading will happen after 1 minute */
    if (xTimerStart(soil_sensor_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start soil sensor timer");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Soil sensor timer started (%d second interval)", 
             SOIL_SENSOR_READ_INTERVAL_SEC);
    ESP_LOGI(TAG, "First reading will occur in %d seconds", 
             SOIL_SENSOR_READ_INTERVAL_SEC);
    
    return ESP_OK;
}

static void app_temp_sensor_update(TimerHandle_t handle)
{
    float temperature = 0.0;
    float humidity = 0.0;
    
    /* Read from DHT22 sensor */
    if (dht_read_float_data(DHT_SENSOR_TYPE, DHT_GPIO, &humidity, &temperature) == ESP_OK) {
        /* Round to 2 decimal places */
        temperature = roundf(temperature * 100.0f) / 100.0f;
        humidity = roundf(humidity * 100.0f) / 100.0f;
        
        ESP_LOGI(TAG, "Read Temp: %.2f C, Humidity: %.2f %%", temperature, humidity);
        
        if (temp_param) {
            esp_rmaker_param_update_and_report(temp_param, esp_rmaker_float(temperature));
        }
        if (humidity_param) {
            esp_rmaker_param_update_and_report(humidity_param, esp_rmaker_float(humidity));
        }
    } else {
        ESP_LOGE(TAG, "Could not read data from DHT sensor");
    }
}

esp_err_t app_temp_sensor_init(void)
{
    /* Cache parameter handles */
    const esp_rmaker_device_t *temp_device = esp_rmaker_node_get_device_by_name(esp_rmaker_get_node(), "Temperature Sensor");
    if (temp_device) {
        temp_param = esp_rmaker_device_get_param_by_name(temp_device, "Temperature");
    }
    
    const esp_rmaker_device_t *hum_device = esp_rmaker_node_get_device_by_name(esp_rmaker_get_node(), "Humidity Sensor");
    if (hum_device) {
        humidity_param = esp_rmaker_device_get_param_by_name(hum_device, "Humidity");
    }

    /* Start a timer to read values periodically */
    TimerHandle_t temp_sensor_timer = xTimerCreate("temp_sensor_update_tm",
                                       (TEMP_SENSOR_UPDATE_INTERVAL_SEC * 1000) / portTICK_PERIOD_MS,
                                       pdTRUE,
                                       NULL,
                                       app_temp_sensor_update);
    
    if (temp_sensor_timer) {
        xTimerStart(temp_sensor_timer, 0);
    } else {
        ESP_LOGE(TAG, "Failed to create temp sensor timer");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}



static void push_btn_cb(void *arg, void *data)
{
    bool new_state = !g_power_state;
    app_driver_set_state(new_state);
    esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_type(pump_device, ESP_RMAKER_PARAM_POWER),
                esp_rmaker_bool(new_state));
}

static void set_power_state(bool target)
{
    /* Inverted logic for Active Low relay: ON(true) -> 0, OFF(false) -> 1 */
    gpio_set_level(OUTPUT_GPIO, !target);
}

void app_driver_init()
{
    button_config_t btn_cfg = {
        .long_press_time = 0,  /* Use default */
        .short_press_time = 0, /* Use default */
    };
    button_gpio_config_t gpio_cfg = {
        .gpio_num = BUTTON_GPIO,
        .active_level = BUTTON_ACTIVE_LEVEL,
        .enable_power_save = false,
    };
    button_handle_t btn_handle = NULL;
    if (iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &btn_handle) == ESP_OK && btn_handle) {
        /* Register a callback for a button single click event */
        iot_button_register_cb(btn_handle, BUTTON_SINGLE_CLICK, NULL, push_btn_cb, NULL);
        /* Register Wi-Fi reset and factory reset functionality on same button */
        app_reset_button_register(btn_handle, WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
    }

    /* Configure power GPIO - Set level BEFORE configuring as output to avoid glitch */
    /* For Active Low relay: Set HIGH first to keep relay OFF during boot */
    gpio_set_direction(OUTPUT_GPIO, GPIO_MODE_INPUT);  // Temporarily set as input
    gpio_set_level(OUTPUT_GPIO, 1);  // Set HIGH (relay OFF for Active Low)
    
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    io_conf.pin_bit_mask = ((uint64_t)1 << OUTPUT_GPIO);
    /* Configure the GPIO */
    gpio_config(&io_conf);
    /* temperature simulation disabled */

    /* NEW: Initialize I2C library (required before any I2C device init) */
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_LOGI(TAG, "I2C library initialized");
    
    /* Start auto-off timer task - Increased stack size for stability */
    xTaskCreate(turnOffSwitchAfterSetInterval, "auto_off_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Auto-off timer task started (default interval: %lu seconds)", switch_off_interval);
}

int IRAM_ATTR app_driver_set_state(bool state)
{
    if(g_power_state != state) {
        g_power_state = state;
        set_power_state(g_power_state);
    }
    return ESP_OK;
}

bool app_driver_get_state(void)
{
    return g_power_state;
}
