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
        PROP_FLAG_READ
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
    
    /* Add status parameters for each sensor */
    const char *status_names[] = {
        PARAM_SENSOR_1_STATUS,
        PARAM_SENSOR_2_STATUS,
        PARAM_SENSOR_3_STATUS,
        PARAM_SENSOR_4_STATUS
    };
    for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
        esp_rmaker_param_t *status_param = esp_rmaker_param_create(
            status_names[i],
            "esp.param.name",
            esp_rmaker_str("Disconnected"),
            PROP_FLAG_READ
        );
        if (status_param) {
            esp_rmaker_device_add_param(soil_monitor_device, status_param);
        }
    }
    ESP_LOGI(TAG, "✓ Status parameters created");
    
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
    esp_rmaker_node_add_device(node, pump_device);

        /* Light, Fan and simulated Temperature Sensor removed to simplify firmware.
           Soil moisture sensors and Pump remain. */
    
    /* NEW: Create Soil Moisture Sensor devices */
    init_soil_sensor_devices(node);

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
