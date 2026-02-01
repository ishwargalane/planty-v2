/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define DEFAULT_SWITCH_POWER        true
#define DEFAULT_LIGHT_POWER         true
#define DEFAULT_LIGHT_BRIGHTNESS    25
#define DEFAULT_FAN_POWER           false
#define DEFAULT_FAN_SPEED           3
#define DEFAULT_TEMPERATURE         25.0
#define REPORTING_PERIOD            60 /* Seconds */

extern esp_rmaker_device_t *pump_device;

/* Consolidated soil moisture monitor device */
extern esp_rmaker_device_t *soil_monitor_device;

/* Parameter name macros for consistency */
#define PARAM_AVERAGE_MOISTURE      "Average Moisture"
#define PARAM_SENSOR_1              "Sensor 1"
#define PARAM_SENSOR_2              "Sensor 2"
#define PARAM_SENSOR_3              "Sensor 3"
#define PARAM_SENSOR_4              "Sensor 4"
#define PARAM_SENSOR_1_STATUS       "Sensor 1 Status"
#define PARAM_SENSOR_2_STATUS       "Sensor 2 Status"
#define PARAM_SENSOR_3_STATUS       "Sensor 3 Status"
#define PARAM_SENSOR_4_STATUS       "Sensor 4 Status"

void app_driver_init(void);
int app_driver_set_state(bool state);
bool app_driver_get_state(void);
/* app_get_current_temperature removed (no simulated temperature) */

/* NEW: Soil sensor functions */
esp_err_t app_soil_sensor_init(void);

/* Auto-off timer functions */
void app_driver_set_switch_off_interval(uint32_t interval_seconds);
uint32_t app_driver_get_switch_off_interval(void);

/* Auto-watering threshold functions */
void app_driver_set_moisture_threshold(uint32_t threshold_percent);
uint32_t app_driver_get_moisture_threshold(void);

/* Automatic mode functions */
void app_driver_set_auto_mode(bool auto_mode);
bool app_driver_get_auto_mode(void);