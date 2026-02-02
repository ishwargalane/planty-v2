/* Multi-Device Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "soil_sensor.h"
#include "sensor_config.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>

#include <app_network.h>
#include <app_insights.h>

#include "app_priv.h"

static const char *TAG = "app_main";

esp_rmaker_device_t *pump_device;

/* Consolidated soil moisture monitor device */
esp_rmaker_device_t *soil_monitor_device;

/* Callback to handle commands received from the RainMaker cloud */
static esp_err_t write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
            const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx)
{
    if (ctx) {
        ESP_LOGI(TAG, "Received write request via : %s", esp_rmaker_device_cb_src_to_str(ctx->src));
    }
    const char *device_name = esp_rmaker_device_get_name(device);
    const char *param_name = esp_rmaker_param_get_name(param);
    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %s for %s - %s",
                val.val.b? "true" : "false", device_name, param_name);
        if (device == pump_device) {
            app_driver_set_state(val.val.b);
        }
    } else if (strcmp(param_name, "Auto-Off Interval") == 0) {
        ESP_LOGI(TAG, "Received value = %d for %s - %s",
                val.val.i, device_name, param_name);
        if (device == pump_device) {
            app_driver_set_switch_off_interval(val.val.i);
        }
    } else if (strcmp(param_name, "Moisture Threshold") == 0) {
        ESP_LOGI(TAG, "Received value = %d for %s - %s",
                val.val.i, device_name, param_name);
        if (device == soil_monitor_device) {
            app_driver_set_moisture_threshold(val.val.i);
        }
    } else if (strcmp(param_name, "Automatic") == 0) {
        ESP_LOGI(TAG, "Received value = %s for %s - %s",
                val.val.b? "true" : "false", device_name, param_name);
        if (device == pump_device) {
            app_driver_set_auto_mode(val.val.b);
        }
    } else if (strcmp(param_name, ESP_RMAKER_DEF_BRIGHTNESS_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %d for %s - %s",
                val.val.i, device_name, param_name);
    } else if (strcmp(param_name, ESP_RMAKER_DEF_SPEED_NAME) == 0) {
        ESP_LOGI(TAG, "Received value = %d for %s - %s",
                val.val.i, device_name, param_name);
    } else {
        /* Silently ignoring invalid params */
        return ESP_OK;
    }
    esp_rmaker_param_update(param, val);
    return ESP_OK;
}

/* Function to create and initialize Temperature and Humidity sensor devices */
static void init_temp_sensor_devices(esp_rmaker_node_t *node)
{
    /* 1. Create Temperature Sensor Device */
    /* Use string literal for device type to avoid macro issues */
    esp_rmaker_device_t *temp_device = esp_rmaker_device_create("Temperature Sensor", "esp.device.temperature-sensor", NULL);
    
     /* Add Name Parameter */
    esp_rmaker_device_add_param(temp_device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Temperature Sensor"));

    /* Add Temperature Parameter */
    esp_rmaker_param_t *temp_param = esp_rmaker_param_create("Temperature", ESP_RMAKER_DEF_TEMPERATURE_NAME, esp_rmaker_float(0.0), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    /* HIDDEN UI or Text? Temperature is usually a value. standard_types has NO UI_TEXT macro mostly, passing string "esp.ui.text" is valid if supported by app, otherwise just don't add UI type or use "esp.ui.hue"? No. */
    /* Actually for Read-only value, we don't strictly need a UI type, the app renders it. */
    /* But to match "esp.ui.text" string: */
    esp_rmaker_param_add_ui_type(temp_param, "esp.ui.text");
    esp_rmaker_device_add_param(temp_device, temp_param);
    esp_rmaker_device_assign_primary_param(temp_device, temp_param);
    
    esp_rmaker_node_add_device(node, temp_device);

    /* 2. Create Humidity Sensor Device */
    /* Using a custom type or generic sensor type for Humidity if not standard definition available easily */
    esp_rmaker_device_t *hum_device = esp_rmaker_device_create("Humidity Sensor", "esp.device.humidity-sensor", NULL);
    
    /* Add Name Parameter */
    esp_rmaker_device_add_param(hum_device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, "Humidity Sensor"));

    /* Add Humidity Parameter */
    esp_rmaker_param_t *hum_param = esp_rmaker_param_create("Humidity", "esp.param.humidity", esp_rmaker_float(0.0), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    esp_rmaker_param_add_ui_type(hum_param, "esp.ui.text");
    esp_rmaker_device_add_param(hum_device, hum_param);
    esp_rmaker_device_assign_primary_param(hum_device, hum_param);
    
    esp_rmaker_node_add_device(node, hum_device);
}

/* Function to create and initialize consolidated soil moisture monitor device */
static void init_soil_sensor_devices(esp_rmaker_node_t *node)
{
    ESP_LOGI(TAG, "Creating consolidated soil moisture monitor device...");
    
    /* Create base temperature sensor device for primary average display */
    soil_monitor_device = esp_rmaker_device_create(
        "Soil Moisture Monitor",
        "esp.device.temperature-sensor",
        NULL
    );
    
    if (soil_monitor_device == NULL) {
        ESP_LOGE(TAG, "Failed to create soil monitor device");
        return;
    }
    
    /* Add device attributes */
    esp_rmaker_device_add_attribute(soil_monitor_device, "sensor_type", "aggregate");
    esp_rmaker_device_add_attribute(soil_monitor_device, "unit", "percent");
    
    /* Create and add average moisture parameter (primary - shows on icon) */
    esp_rmaker_param_t *avg_param = esp_rmaker_param_create(
        PARAM_AVERAGE_MOISTURE,
        ESP_RMAKER_DEF_TEMPERATURE_NAME,
        esp_rmaker_float(0.0),
        PROP_FLAG_READ | PROP_FLAG_TIME_SERIES
    );
    if (avg_param) {
        esp_rmaker_device_add_param(soil_monitor_device, avg_param);
        esp_rmaker_device_assign_primary_param(soil_monitor_device, avg_param);
        esp_rmaker_param_add_ui_type(avg_param, "esp.ui.slider");
        esp_rmaker_param_add_bounds(avg_param,
                                   esp_rmaker_float(0.0),
                                   esp_rmaker_float(100.0),
                                   esp_rmaker_float(0.1));
        ESP_LOGI(TAG, "✓ Average moisture parameter created");
    }
    
    /* Add individual sensor parameters */
    const char *sensor_names[] = {PARAM_SENSOR_1, PARAM_SENSOR_2, PARAM_SENSOR_3, PARAM_SENSOR_4};
    for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
        esp_rmaker_param_t *sensor_param = esp_rmaker_param_create(
            sensor_names[i],
            ESP_RMAKER_DEF_TEMPERATURE_NAME,
            esp_rmaker_float(0.0),
            PROP_FLAG_READ
        );
        if (sensor_param) {
            esp_rmaker_device_add_param(soil_monitor_device, sensor_param);
            esp_rmaker_param_add_ui_type(sensor_param, "esp.ui.slider");
            esp_rmaker_param_add_bounds(sensor_param,
                                       esp_rmaker_float(0.0),
                                       esp_rmaker_float(100.0),
                                       esp_rmaker_float(0.1));
        }
    }
    ESP_LOGI(TAG, "✓ Individual sensor parameters created");
    
    /* Add Moisture Threshold parameter (slider: 0-100%, default 30%) */
    esp_rmaker_param_t *threshold_param = esp_rmaker_param_create(
        "Moisture Threshold", NULL, esp_rmaker_int(30), PROP_FLAG_READ | PROP_FLAG_WRITE);
    if (threshold_param) {
        esp_rmaker_param_add_ui_type(threshold_param, "esp.ui.slider");
        esp_rmaker_param_add_bounds(threshold_param, esp_rmaker_int(0), esp_rmaker_int(100), esp_rmaker_int(1));
        esp_rmaker_device_add_param(soil_monitor_device, threshold_param);
        ESP_LOGI(TAG, "✓ Moisture threshold parameter created");
    }
    
    /* Add device to node */
    esp_err_t err = esp_rmaker_node_add_device(node, soil_monitor_device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add soil monitor device to node: %s", esp_err_to_name(err));
        return;
    }
    
    ESP_LOGI(TAG, "✓ Soil moisture monitor device setup complete");
}

void app_main()
{
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    app_driver_init();
    app_driver_set_state(DEFAULT_SWITCH_POWER);

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_network_init();
    
    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_network_init() but before app_network_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Multi Device", "Multi Device");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Create a Pump device (reused from Switch) and add the relevant parameters to it */
    pump_device = esp_rmaker_switch_device_create("Pump", NULL, DEFAULT_SWITCH_POWER);
    esp_rmaker_device_add_cb(pump_device, write_cb, NULL);
    esp_rmaker_device_add_attribute(pump_device, "device", "pump");
    esp_rmaker_device_add_attribute(pump_device, "Serial Number", "PN-001");
    
    /* Add Auto-Off Interval parameter (slider: 1-300 seconds, default 10) */
    esp_rmaker_param_t *auto_off_param = esp_rmaker_param_create(
        "Auto-Off Interval", NULL, esp_rmaker_int(10), PROP_FLAG_READ | PROP_FLAG_WRITE);
    if (auto_off_param) {
        esp_rmaker_param_add_ui_type(auto_off_param, "esp.ui.slider");
        esp_rmaker_param_add_bounds(auto_off_param, esp_rmaker_int(1), esp_rmaker_int(300), esp_rmaker_int(1));
        esp_rmaker_device_add_param(pump_device, auto_off_param);
    }
    
    /* Add Automatic Mode parameter (toggle: default true) */
    esp_rmaker_param_t *auto_mode_param = esp_rmaker_param_create(
        "Automatic", NULL, esp_rmaker_bool(true), PROP_FLAG_READ | PROP_FLAG_WRITE);
    if (auto_mode_param) {
        esp_rmaker_param_add_ui_type(auto_mode_param, "esp.ui.toggle");
        esp_rmaker_device_add_param(pump_device, auto_mode_param);
    }
    
    esp_rmaker_node_add_device(node, pump_device);

        /* Light, Fan and simulated Temperature Sensor removed to simplify firmware.
           Soil moisture sensors and Pump remain. */
    
    /* NEW: Create Soil Moisture Sensor devices */
    init_soil_sensor_devices(node);
    
    /* NEW: Create config Temperature & Humidity devices */
    init_temp_sensor_devices(node);

    /* Initialize soil sensor hardware and start periodic reading */
    ESP_ERROR_CHECK(app_soil_sensor_init());
    
    /* Initialize temperature sensor */
    ESP_ERROR_CHECK(app_temp_sensor_init());

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable timezone service which will be require for setting appropriate timezone
     * from the phone apps for scheduling to work correctly.
     * For more information on the various ways of setting timezone, please check
     * https://rainmaker.espressif.com/docs/time-service.html.
     */
    esp_rmaker_timezone_service_enable();

    /* Enable scheduling. */
    esp_rmaker_schedule_enable();

    /* Enable Scenes */
    esp_rmaker_scenes_enable();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_network_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    /* Normal operation: exit app_main after services started. */
    return;
}
