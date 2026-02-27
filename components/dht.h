#pragma once

/**
 * @file dht.h
 * @brief Minimal DHT11/DHT22 single-wire driver for ESP8266 ESP-IDF.
 *
 * The ESP-IDF SDK for ESP8266 does not ship a DHT driver in its components,
 * so we implement a lightweight one here.
 *
 * Protocol summary (DHT11):
 *   Host pulls line LOW for ≥18 ms (start signal)
 *   Host releases line; it rises via pull-up
 *   DHT responds: 80 µs LOW then 80 µs HIGH
 *   40 bits follow: each bit = 50 µs LOW + (26-28 µs=0 or 70 µs=1) HIGH
 *   5 bytes: RH_int, RH_dec, T_int, T_dec, checksum
 */

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DHT_TYPE_DHT11 = 11,
    DHT_TYPE_DHT22 = 22,
} dht_sensor_type_t;

/**
 * @brief Read temperature and humidity from a DHT sensor.
 *
 * @param sensor_type  DHT_TYPE_DHT11 or DHT_TYPE_DHT22
 * @param gpio_num     GPIO pin connected to DHT data line
 * @param humidity     Output: relative humidity in %
 * @param temperature  Output: temperature in °C
 * @return ESP_OK on success, ESP_ERR_TIMEOUT or ESP_ERR_INVALID_CRC on failure
 */
esp_err_t dht_read_float_data(dht_sensor_type_t sensor_type,
                               gpio_num_t gpio_num,
                               float *humidity,
                               float *temperature);

#ifdef __cplusplus
}
#endif
