/*
 * Soil Moisture Sensor Driver
 * Uses ADS1115 ADC to read capacitive soil moisture sensors
 */
#ifndef SOIL_SENSOR_H
#define SOIL_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initialize the ADS1115 ADC for soil moisture sensing
 * 
 * This must be called after i2cdev_init() and before any read operations.
 * Configures the ADS1115 for single-shot mode with ±4.096V range.
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t soil_sensor_init(void);

/**
 * @brief Read raw ADC value from a specific channel
 * 
 * @param channel Channel number (0-3) corresponding to A0-A3 on ADS1115
 * @param raw_value Pointer to store raw 16-bit ADC value
 * @param voltage Pointer to store calculated voltage (can be NULL if not needed)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t soil_sensor_read_channel(uint8_t channel, int16_t *raw_value, float *voltage);

/**
 * @brief Convert raw ADC value to moisture percentage
 * 
 * Uses calibration values from sensor_config.h to map ADC reading to 0-100%
 * 
 * @param raw_value Raw 16-bit ADC value
 * @return Moisture percentage (0.0 - 100.0)
 */
float soil_sensor_raw_to_percentage(int16_t raw_value);

/**
 * @brief Read all connected soil sensors and return moisture percentages
 * 
 * @param sensor_values Array to store moisture percentages (must have space for num_sensors floats)
 * @param num_sensors Number of sensors to read (1-4)
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t soil_sensor_read_all(float *sensor_values, uint8_t num_sensors);

/**
 * @brief Cleanup and free ADS1115 resources
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t soil_sensor_deinit(void);

/**
 * @brief Run calibration mode (for initial setup)
 * 
 * Continuously reads and displays raw values from all channels.
 * Use this to determine DRY and WET calibration values.
 * This function never returns - use only during calibration!
 */
void soil_sensor_calibration_mode(void);

#endif // SOIL_SENSOR_H