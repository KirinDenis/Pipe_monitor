/**
 * @file dht.c
 * @brief DHT11 / DHT22 single-wire driver for ESP8266 ESP-IDF.
 *
 * Timing is achieved by busy-wait loops using xthal_get_ccount() which
 * counts CPU cycles at 80 MHz. We disable interrupts during the bit-read
 * loop to prevent jitter from corrupting the time measurements.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "xtensa/core-macros.h"   /* xthal_get_ccount */
#include "dht.h"

static const char *TAG = "dht";

/* CPU runs at 80 MHz in low-power mode (saves ~15 mA vs 160 MHz) */
#define CPU_MHZ         80
#define US_TO_CYCLES(us) ((us) * CPU_MHZ)

/* -------------------------------------------------------------------------
 * Busy-wait helpers
 * ------------------------------------------------------------------------- */

/** Wait until GPIO level becomes `expected` or timeout_us elapses.
 *  Returns remaining time budget (>0) on success, 0 on timeout. */
static int wait_for_level(gpio_num_t pin, int expected, int timeout_us)
{
    uint32_t deadline = xthal_get_ccount() + US_TO_CYCLES(timeout_us);
    while (gpio_get_level(pin) != expected) {
        if ((int32_t)(deadline - xthal_get_ccount()) <= 0) return 0;
    }
    return 1;
}

/** Measure how long (in µs) a GPIO level lasts, up to timeout_us. */
static int measure_level_us(gpio_num_t pin, int level, int timeout_us)
{
    uint32_t start    = xthal_get_ccount();
    uint32_t deadline = start + US_TO_CYCLES(timeout_us);
    while (gpio_get_level(pin) == level) {
        if ((int32_t)(deadline - xthal_get_ccount()) <= 0) return timeout_us + 1;
    }
    uint32_t elapsed = xthal_get_ccount() - start;
    return (int)(elapsed / CPU_MHZ);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

esp_err_t dht_read_float_data(dht_sensor_type_t sensor_type,
                               gpio_num_t gpio_num,
                               float *humidity,
                               float *temperature)
{
    uint8_t data[5] = {0};

    /* ------------------------------------------------------------------
     * Step 1: Send start signal
     *   Drive line LOW for 18 ms (DHT11 spec: ≥18 ms)
     *   Then release (input mode, pull-up keeps line HIGH)
     * ------------------------------------------------------------------ */
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(gpio_num, 0);
    vTaskDelay(pdMS_TO_TICKS(20));   /* 20 ms > 18 ms minimum */
    gpio_set_level(gpio_num, 1);

    /* Switch to input; external 4.7kΩ pull-up will keep line HIGH */
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);

    /* ------------------------------------------------------------------
     * Step 2: Wait for DHT response (80 µs LOW + 80 µs HIGH)
     * ------------------------------------------------------------------ */
    /* Line should still be HIGH for a brief moment, then DHT pulls LOW */
    if (!wait_for_level(gpio_num, 0, 40)) {
        ESP_LOGW(TAG, "No response from DHT (waiting for LOW)");
        return ESP_ERR_TIMEOUT;
    }
    /* DHT holds LOW ~80 µs */
    if (!wait_for_level(gpio_num, 1, 100)) {
        ESP_LOGW(TAG, "Timeout waiting for DHT HIGH ack");
        return ESP_ERR_TIMEOUT;
    }
    /* DHT holds HIGH ~80 µs */
    if (!wait_for_level(gpio_num, 0, 100)) {
        ESP_LOGW(TAG, "Timeout waiting for data start LOW");
        return ESP_ERR_TIMEOUT;
    }

    /* ------------------------------------------------------------------
     * Step 3: Read 40 bits (5 bytes)
     *   Each bit:  50 µs LOW preamble
     *              then HIGH: 26-28 µs = '0',  70 µs = '1'
     *
     *   Disable interrupts during the tight timing loop.
     * ------------------------------------------------------------------ */
    portDISABLE_INTERRUPTS();

    for (int i = 0; i < 40; i++) {
        /* Wait for the LOW preamble to end */
        if (!wait_for_level(gpio_num, 1, 70)) {
            portENABLE_INTERRUPTS();
            ESP_LOGW(TAG, "Timeout on bit %d LOW->HIGH", i);
            return ESP_ERR_TIMEOUT;
        }
        /* Measure HIGH pulse duration to determine bit value */
        int high_us = measure_level_us(gpio_num, 1, 100);
        data[i / 8] <<= 1;
        if (high_us > 40) {
            data[i / 8] |= 1;  /* long HIGH = '1' */
        }
        /* (short HIGH = '0', bit already 0 from shift) */
    }

    portENABLE_INTERRUPTS();

    /* ------------------------------------------------------------------
     * Step 4: Verify checksum
     *   Byte 4 = (byte0 + byte1 + byte2 + byte3) & 0xFF
     * ------------------------------------------------------------------ */
    uint8_t checksum = (data[0] + data[1] + data[2] + data[3]) & 0xFF;
    if (checksum != data[4]) {
        ESP_LOGW(TAG, "CRC fail: computed 0x%02X got 0x%02X", checksum, data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    /* ------------------------------------------------------------------
     * Step 5: Decode
     *   DHT11: integer only. data[0]=RH%, data[2]=T°C (data[1],[3] are 0)
     *   DHT22: 16-bit big-endian signed/unsigned with 0.1 resolution
     * ------------------------------------------------------------------ */
    if (sensor_type == DHT_TYPE_DHT11) {
        *humidity    = (float)data[0];
        *temperature = (float)data[2];
    } else {
        /* DHT22 */
        uint16_t raw_h = ((uint16_t)data[0] << 8) | data[1];
        uint16_t raw_t = ((uint16_t)data[2] << 8) | data[3];
        *humidity = raw_h * 0.1f;
        if (raw_t & 0x8000) {
            *temperature = -((raw_t & 0x7FFF) * 0.1f);
        } else {
            *temperature = raw_t * 0.1f;
        }
    }

    ESP_LOGD(TAG, "Raw bytes: %02X %02X %02X %02X %02X",
             data[0], data[1], data[2], data[3], data[4]);
    return ESP_OK;
}
