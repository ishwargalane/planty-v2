#include <stdio.h>
#include <string.h> 
#include <esp_log.h>
#include <i2cdev.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sensor_config.h"

static const char *TAG = "I2C_SCANNER";

void i2c_scanner(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║       I2C BUS SCANNER                      ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "Scanning I2C bus...");
    ESP_LOGI(TAG, "SDA: GPIO %d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "SCL: GPIO %d", I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "");
    
    int devices_found = 0;
    
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_dev_t test_dev;
        memset(&test_dev, 0, sizeof(i2c_dev_t));
        
        esp_err_t ret = i2c_dev_create_mutex(&test_dev);
        if (ret != ESP_OK) {
            continue;
        }
        
        test_dev.port = I2C_MASTER_NUM;
        test_dev.addr = addr;
        test_dev.cfg.sda_io_num = I2C_MASTER_SDA_IO;
        test_dev.cfg.scl_io_num = I2C_MASTER_SCL_IO;
        test_dev.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;
        
        // Try to probe the device
        ret = i2c_dev_probe(&test_dev, I2C_DEV_WRITE);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address: 0x%02X", addr);
            
            // Check if it's the ADS1115
            if (addr == 0x48) {
                ESP_LOGI(TAG, "  └─> This is ADS1115 (0x48) ✓");
            } else if (addr == 0x49) {
                ESP_LOGI(TAG, "  └─> This could be ADS1115 (0x49)");
            }
            devices_found++;
        }
        
        i2c_dev_delete_mutex(&test_dev);
    }
    
    ESP_LOGI(TAG, "");
    if (devices_found == 0) {
        ESP_LOGE(TAG, "❌ NO I2C DEVICES FOUND!");
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "Possible issues:");
        ESP_LOGE(TAG, "  1. ADS1115 not powered (check VCC and GND)");
        ESP_LOGE(TAG, "  2. Wrong SDA/SCL pins");
        ESP_LOGE(TAG, "  3. Bad wiring or loose connections");
        ESP_LOGE(TAG, "  4. ADS1115 hardware fault");
        ESP_LOGE(TAG, "  5. Missing pull-up resistors (usually on module)");
    } else {
        ESP_LOGI(TAG, "✓ Found %d I2C device(s)", devices_found);
    }
    ESP_LOGI(TAG, "════════════════════════════════════════════");
    ESP_LOGI(TAG, "");
}