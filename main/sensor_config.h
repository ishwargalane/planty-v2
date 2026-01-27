/*
 * Soil Moisture Sensor Configuration
 */
#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <driver/i2c.h>
#include <ads111x.h>

/* I2C Configuration */
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   21          // Change based on your ESP32 board
#define I2C_MASTER_SCL_IO   22          // Change based on your ESP32 board
#define I2C_MASTER_FREQ_HZ  100000      // 100kHz

/* ADS1115 Configuration */
#define ADS1115_ADDR        ADS111X_ADDR_GND  // Address when ADDR pin connected to GND (0x48)

/* Number of soil moisture sensors connected (1-4) */
#define NUM_SOIL_SENSORS    4

/* Sampling Configuration */
#define SOIL_SENSOR_READ_INTERVAL_SEC   (1 * 60)  // 15 minutes in seconds

/* Moisture Calibration Values 
 * IMPORTANT: These are placeholder values. You MUST calibrate these!
 * Run calibration mode to get actual values for your sensors.
 * 
 * Typical values:
 * - Dry sensor in air: 20000-26000
 * - Wet sensor in water: 10000-15000
 * 
 * Higher value = drier (less conductivity)
 * Lower value = wetter (more conductivity)
 */
#define MOISTURE_DRY_VALUE   17500    // ADC value when completely dry (in air)
#define MOISTURE_WET_VALUE   7600    // ADC value when fully wet (in water)

#endif // SENSOR_CONFIG_H