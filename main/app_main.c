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

/* NEW: Array of soil moisture sensor devices */
esp_rmaker_device_t *soil_sensor_devices[NUM_SOIL_SENSORS];

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

/* Function to create and initialize soil moisture sensor devices */
/* Function to create and initialize soil moisture sensor devices */
static void init_soil_sensor_devices(esp_rmaker_node_t *node)
{
    char device_name[32];
    char serial_number[32];
    
    ESP_LOGI(TAG, "Creating %d soil moisture sensor devices...", NUM_SOIL_SENSORS);
    
    for (int i = 0; i < NUM_SOIL_SENSORS; i++) {
        // Create unique device name
        snprintf(device_name, sizeof(device_name), "Soil Moisture %d", i + 1);
        snprintf(serial_number, sizeof(serial_number), "SM-%d", 1000 + i);
        
        ESP_LOGI(TAG, "Creating device %d: %s", i, device_name);
        
        /* Create temperature sensor device */
        soil_sensor_devices[i] = esp_rmaker_temp_sensor_device_create(
            device_name,
            NULL,  // No private data
            0.0f   // Initial value
        );
        
        if (soil_sensor_devices[i] == NULL) {
            ESP_LOGE(TAG, "FAILED to create soil sensor device %d", i);
            continue; // Skip this device
        }
        
        ESP_LOGI(TAG, "Device %d created successfully at %p", i, soil_sensor_devices[i]);
        
        // Add device to the Rainmaker node
        esp_err_t err = esp_rmaker_node_add_device(node, soil_sensor_devices[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add device %d to node: %s", i, esp_err_to_name(err));
            continue;
        }
        
        // Add custom attributes
        esp_rmaker_device_add_attribute(soil_sensor_devices[i], 
                                       "sensor_type", "soil_moisture");
        esp_rmaker_device_add_attribute(soil_sensor_devices[i], 
                                       "unit", "percent");
        esp_rmaker_device_add_attribute(soil_sensor_devices[i], 
                                       "Serial Number", serial_number);
        
        // Get the temperature parameter
        esp_rmaker_param_t *moisture_param = esp_rmaker_device_get_param_by_name(
            soil_sensor_devices[i], 
            ESP_RMAKER_DEF_TEMPERATURE_NAME
        );
        
        if (moisture_param) {
            // Add UI customization
            esp_rmaker_param_add_ui_type(moisture_param, "esp.ui.slider");
            esp_rmaker_param_add_bounds(moisture_param, 
                                       esp_rmaker_float(0.0),
                                       esp_rmaker_float(100.0),
                                       esp_rmaker_float(0.1));
            ESP_LOGI(TAG, "Device %d parameter configured", i);
        } else {
            ESP_LOGW(TAG, "Could not get parameter for device %d", i);
        }

        /* Add a companion Status parameter (string) to indicate connectivity */
        esp_rmaker_param_t *status_param = esp_rmaker_param_create(
            "Status",
            "string",
            esp_rmaker_str("Connected"),
            PROP_FLAG_READ
        );
        if (status_param) {
            esp_rmaker_device_add_param(soil_sensor_devices[i], status_param);
        }
        
        ESP_LOGI(TAG, "✓ Soil moisture sensor device %d setup complete", i);
    }
    
    ESP_LOGI(TAG, "All soil moisture sensor devices created");
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
