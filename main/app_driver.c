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
#include "i2c_scanner.h"

#include <i2cdev.h>
#include "soil_sensor.h"
#include "sensor_config.h"

#include <iot_button.h>
#include <button_gpio.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h>
#include <esp_rmaker_standard_params.h>

#include <app_reset.h>

#include <led_indicator.h>
#include <led_convert.h>
#ifdef CONFIG_LED_TYPE_RGB
#include <led_indicator_rgb.h>
#elif defined(CONFIG_LED_TYPE_WS2812)
#include <led_indicator_strips.h>
#endif

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

/* LED Indicator handle */
static led_indicator_handle_t g_led_indicator = NULL;

/* Soil Sensor handle */
static TimerHandle_t soil_sensor_timer;

/* Cached parameter pointers for efficient updates */
static esp_rmaker_param_t *avg_moisture_param = NULL;
static esp_rmaker_param_t *sensor_params[NUM_SOIL_SENSORS] = {NULL};

static esp_err_t app_indicator_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!g_led_indicator) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Convert percentage (0-100) to 0-255 range */
    uint8_t r_255 = (red * 255) / 100;
    uint8_t g_255 = (green * 255) / 100;
    uint8_t b_255 = (blue * 255) / 100;

    return led_indicator_set_rgb(g_led_indicator, SET_IRGB(MAX_INDEX, r_255, g_255, b_255));
}

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

    /* Calculate average from all sensors */
    float sum = 0.0f;
    int valid_count = 0;
    
    for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
        if (sensor_values[i] != -1.0f) {
            sum += sensor_values[i];
            valid_count++;
        }
    }
    
    float average = (valid_count > 0) ? (sum / valid_count) : 0.0f;
    
    /* Update average moisture parameter */
    if (avg_moisture_param) {
        esp_rmaker_param_update_and_report(avg_moisture_param, esp_rmaker_float(average));
        ESP_LOGI(TAG, "✓ Average Moisture: %.1f%% (%d/%d sensors)", 
                 average, valid_count, NUM_SOIL_SENSORS);
    }
    
    /* Update individual sensor parameters */
    for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
        if (sensor_values[i] != -1.0f && sensor_params[i]) {
            esp_rmaker_param_update_and_report(sensor_params[i], esp_rmaker_float(sensor_values[i]));
            ESP_LOGI(TAG, "✓ Sensor %d: %.1f%%", i + 1, sensor_values[i]);
        }
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

/* app_get_current_temperature and app_sensor_init removed (no simulated temperature) */

static void app_indicator_set(bool state)
{
#ifdef CONFIG_LED_TYPE_NONE
    /* No LED hardware - nothing to do */
    return;
#endif

    if (state) {
        app_indicator_set_rgb(DEFAULT_RED, DEFAULT_GREEN, DEFAULT_BLUE);
    } else {
        if (g_led_indicator) {
            led_indicator_set_brightness(g_led_indicator, 0);
        }
    }
}

static void app_indicator_init(void)
{
#ifdef CONFIG_LED_TYPE_RGB
    /* Use RGB LED (3-pin RGB LED) */
    led_indicator_rgb_config_t rgb_config = {
        .timer_inited = false,
        .timer_num = LEDC_TIMER_0,
        .red_gpio_num = CONFIG_RGB_LED_RED_GPIO,
        .green_gpio_num = CONFIG_RGB_LED_GREEN_GPIO,
        .blue_gpio_num = CONFIG_RGB_LED_BLUE_GPIO,
        .red_channel = LEDC_CHANNEL_0,
        .green_channel = LEDC_CHANNEL_1,
        .blue_channel = LEDC_CHANNEL_2,
    };
#ifdef CONFIG_RGB_LED_ACTIVE_LEVEL_HIGH
    rgb_config.is_active_level_high = true;
#else
    rgb_config.is_active_level_high = false;
#endif

    led_indicator_config_t config = {
        .blink_lists = NULL,
        .blink_list_num = 0,
    };

    esp_err_t err = led_indicator_new_rgb_device(&config, &rgb_config, &g_led_indicator);
#elif defined(CONFIG_LED_TYPE_WS2812)
    /* Use LED Strip (WS2812) */
    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg = {
            .strip_gpio_num = CONFIG_WS2812_LED_GPIO,
            .max_leds = CONFIG_WS2812_LED_COUNT,
            .led_model = LED_MODEL_WS2812,
            .flags.invert_out = false,
        },
        .led_strip_driver = LED_STRIP_RMT,
        .led_strip_rmt_cfg = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = 10 * 1000 * 1000, /* 10 MHz */
            .flags.with_dma = false,
        },
    };

    led_indicator_config_t config = {
        .blink_lists = NULL,
        .blink_list_num = 0,
    };

    esp_err_t err = led_indicator_new_strips_device(&config, &strips_config, &g_led_indicator);
#else
    /* No LED hardware - g_led_indicator remains NULL */
    esp_err_t err = ESP_OK;
#endif

    if (err != ESP_OK) {
        return;
    }
    app_indicator_set(g_power_state);
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
    gpio_set_level(OUTPUT_GPIO, target);
    app_indicator_set(target);
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

    /* Configure power */
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 1,
    };
    io_conf.pin_bit_mask = ((uint64_t)1 << OUTPUT_GPIO);
    /* Configure the GPIO */
    gpio_config(&io_conf);
    app_indicator_init();
    /* temperature simulation disabled */

    /* NEW: Initialize I2C library (required before any I2C device init) */
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_LOGI(TAG, "I2C library initialized");
    
    /* Add delay and visual separator */
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "    RUNNING I2C SCANNER IN 2 SECONDS   ");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second delay
    
    /* NEW: Scan I2C bus to verify ADS1115 connection */
    i2c_scanner();  

    /* NEW: Initialize soil moisture sensors */
    ESP_ERROR_CHECK(app_soil_sensor_init());
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
