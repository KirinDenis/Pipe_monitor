/**
 * @file main.c
 * @brief Hot water pipe temperature monitor for ESP8266 (ESP-IDF)
 *
 * Hardware:
 *   - ESP8266 module (NodeMCU or bare module)
 *   - DHT11 temperature sensor on GPIO4 (4.7kΩ pull-up to 3.3V required)
 *   - Red LED on GPIO12 (330Ω series resistor to GND)
 *   - Active buzzer on GPIO14 (active buzzer = buzzes when 3.3V applied)
 *   - GPIO16 MUST be connected to RST for deep sleep wake-up to work!
 *   - Power: 9V "Krona" → AMS1117-3.3 LDO → 3.3V rail
 *
 * Power budget (rough):
 *   Deep sleep current:    ~20 µA
 *   Active time:           ~80 mA for ~3 seconds per wake
 *   Average per 5-min cycle: ≈ 0.02 + (80 * 3/300) ≈ 0.82 mA mean
 *   Krona (500 mAh) lifetime: ≈ 500/0.82 ≈ 600 hours ≈ 25 days
 *   (real-world less due to LDO quiescent current and file-write spikes)
 *
 * Algorithm (see evaluate_pipe_hot() for full explanation):
 *   Collects up to 7 days of 5-minute readings.
 *   Declares pipe "hot" when current temperature exceeds the 75th
 *   percentile of historical readings AND is above 40°C absolute floor.
 *   This self-calibrates to the specific building's water temperature range.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "driver/gpio.h"

#include "esp_spiffs.h"
#include "esp_vfs.h"

#include "dht.h"
#include "config.h"

static const char *TAG = "pipe_monitor";

/* Deep sleep duration: 5 minutes in microseconds */
#define SLEEP_DURATION_US  (5ULL * 60ULL * 1000000ULL)

/* =========================================================================
 * Forward declarations
 * ========================================================================= */
static void      disable_wifi(void);
static esp_err_t init_spiffs(void);
static uint32_t  read_counter(void);
static void      write_counter(uint32_t counter);
static void      append_record(uint32_t counter, float temp);
static int       load_history(float *out_temps, uint32_t *out_counters, int max_records);
static bool      evaluate_pipe_hot(float current_temp, float *history, int history_len);
static void      led_on(void);
static void      led_off(void);
static void      buzzer_beep(int times, int on_ms, int off_ms);
static void      go_to_deep_sleep(void);

/* =========================================================================
 * ENTRY POINT — called on every boot (including wake from deep sleep)
 * ========================================================================= */
void app_main(void)
{
    /* ------------------------------------------------------------------
     * 1. NVS init (required by WiFi subsystem even when we disable it)
     * ------------------------------------------------------------------ */
    ESP_ERROR_CHECK(nvs_flash_init());

    /* ------------------------------------------------------------------
     * 2. Kill WiFi radio immediately.
     *    This is the biggest single power saving: WiFi RF block draws
     *    ~80 mA when active. We must suppress it before it auto-starts.
     * ------------------------------------------------------------------ */
    disable_wifi();

    /* ------------------------------------------------------------------
     * 3. Configure GPIO outputs
     * ------------------------------------------------------------------ */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_LED) | (1ULL << PIN_BUZZER),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    /* Ensure outputs start LOW (LED off, buzzer silent) */
    gpio_set_level(PIN_LED,    0);
    gpio_set_level(PIN_BUZZER, 0);

    /* ------------------------------------------------------------------
     * 4. Mount SPIFFS filesystem
     *    If mount fails we can't log data, so just go back to sleep.
     * ------------------------------------------------------------------ */
    if (init_spiffs() != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS failed — sleeping");
        go_to_deep_sleep();
        return; /* unreachable, keeps compiler happy */
    }

    /* ------------------------------------------------------------------
     * 5. Increment persistent wake counter
     *
     *    WHY: Deep sleep resets the CPU; there is no monotonic clock that
     *    survives across sleep cycles in ESP8266 (RTC memory is cleared on
     *    some reset types). We keep a simple counter in a file instead.
     *    This counter lets us reconstruct the time axis of the log after
     *    downloading the CSV: record_time = counter * 5_minutes.
     * ------------------------------------------------------------------ */
    uint32_t wake_counter = read_counter() + 1;
    write_counter(wake_counter);
    ESP_LOGI(TAG, "=== Wake #%lu ===", (unsigned long)wake_counter);

    /* ------------------------------------------------------------------
     * 6. Wait for DHT11 to stabilise, then read temperature
     *
     *    DHT11 performs its first measurement 1 second after power-on.
     *    After deep sleep the chip fully reboots, so we always wait.
     *    Using 1200 ms gives comfortable margin for the 1 s requirement.
     * ------------------------------------------------------------------ */
    vTaskDelay(pdMS_TO_TICKS(DHT_STABILISE_MS));

    float temperature = NAN;
    float humidity    = NAN;
    esp_err_t dht_err = dht_read_float_data(DHT_TYPE_DHT11, PIN_DHT,
                                             &humidity, &temperature);
    if (dht_err != ESP_OK) {
        ESP_LOGW(TAG, "DHT11 read error %d — will skip this sample", dht_err);
        temperature = NAN;
    } else {
        ESP_LOGI(TAG, "Sensor: %.0f°C  RH:%.0f%%", temperature, humidity);
    }

    /* ------------------------------------------------------------------
     * 7. Append reading to circular log file (if read succeeded)
     * ------------------------------------------------------------------ */
    if (!isnan(temperature)) {
        append_record(wake_counter, temperature);
    }

    /* ------------------------------------------------------------------
     * 8. Load full history and evaluate pipe state
     * ------------------------------------------------------------------ */
    static float    history_temps[MAX_RECORDS];    /* static: avoid stack */
    static uint32_t history_counters[MAX_RECORDS];

    int history_len = load_history(history_temps, history_counters, MAX_RECORDS);

    bool pipe_is_hot = false;
    if (!isnan(temperature)) {
        if (history_len < MIN_HISTORY_RECORDS) {
            ESP_LOGI(TAG, "Collecting baseline: %d / %d records",
                     history_len, MIN_HISTORY_RECORDS);
        } else {
            pipe_is_hot = evaluate_pipe_hot(temperature, history_temps, history_len);
        }
    }

    ESP_LOGI(TAG, "Decision: pipe is %s", pipe_is_hot ? "HOT ✓" : "cold ✗");

    /* ------------------------------------------------------------------
     * 9. Signal user via LED and buzzer
     *
     *    Sequence when HOT:
     *      - Turn LED on
     *      - Beep buzzer BUZZER_BEEP_COUNT times
     *      - Keep LED on for LED_VISIBLE_MS so user can see it
     *      - Turn LED off (it will also go off when we enter deep sleep)
     *
     *    Note: we cannot keep the LED on *during* deep sleep without
     *    external circuitry (e.g. a latch). For a battery device the
     *    5-minute beep-on-wake is the primary notification mechanism.
     * ------------------------------------------------------------------ */
    if (pipe_is_hot) {
        led_on();
        buzzer_beep(BUZZER_BEEP_COUNT, BUZZER_ON_MS, BUZZER_OFF_MS);
        vTaskDelay(pdMS_TO_TICKS(LED_VISIBLE_MS));
        led_off();
    }

    /* ------------------------------------------------------------------
     * 10. Cleanly unmount filesystem before sleep
     *     This flushes pending writes and closes all open files.
     * ------------------------------------------------------------------ */
    esp_vfs_spiffs_unregister(NULL);

    /* ------------------------------------------------------------------
     * 11. Enter deep sleep for 5 minutes
     *     Remember: GPIO16 must be connected to RST!
     * ------------------------------------------------------------------ */
    go_to_deep_sleep();
}

/* =========================================================================
 * WiFi
 * ========================================================================= */

/**
 * @brief Disable WiFi modem completely to save power.
 *
 * On ESP8266, the WiFi/RF block can draw 60–170 mA when active.
 * Even in modem-sleep mode it periodically wakes. Setting WIFI_MODE_NULL
 * and calling esp_wifi_stop() ensures the radio is fully off.
 */
static void disable_wifi(void)
{
    /* Stop may fail gracefully if WiFi was never started — that's fine */
    esp_wifi_stop();
    esp_wifi_set_mode(WIFI_MODE_NULL);
    ESP_LOGI(TAG, "WiFi disabled");
}

/* =========================================================================
 * SPIFFS
 * ========================================================================= */

static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = "/spiffs",
        .partition_label        = NULL,   /* first 'spiffs' partition in partition table */
        .max_files              = 5,
        .format_if_mount_failed = true,   /* auto-format on first boot */
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS register: %s", esp_err_to_name(ret));
    } else {
        size_t total = 0, used = 0;
        esp_spiffs_info(NULL, &total, &used);
        ESP_LOGI(TAG, "SPIFFS: %d / %d bytes used", used, total);
    }
    return ret;
}

/* =========================================================================
 * Persistent wake counter
 *
 * File: /spiffs/counter.txt  — contains a single ASCII decimal number.
 * Stored as text so it can be read/reset via serial terminal if needed.
 * ========================================================================= */

static uint32_t read_counter(void)
{
    FILE *f = fopen("/spiffs/counter.txt", "r");
    if (!f) return 0;  /* file missing = first boot */
    uint32_t val = 0;
    fscanf(f, "%lu", (unsigned long *)&val);
    fclose(f);
    return val;
}

static void write_counter(uint32_t counter)
{
    FILE *f = fopen("/spiffs/counter.txt", "w");
    if (!f) { ESP_LOGE(TAG, "Cannot write counter.txt"); return; }
    fprintf(f, "%lu\n", (unsigned long)counter);
    fclose(f);
}

/* =========================================================================
 * Circular temperature log
 *
 * File: /spiffs/log.csv
 * Format (one record per line):
 *   <wake_counter>,<temperature_celsius>\n
 *   e.g.:  1042,47.0
 *
 * The file holds at most MAX_RECORDS lines (7 days × 288 samples/day).
 * When full, the oldest record is evicted. The counter field lets you
 * determine the time of each record when downloading the log:
 *   UTC_time = deployment_start + counter * 5_minutes
 *
 * Implementation note: we load the entire file into RAM, modify, rewrite.
 * With 2016 lines of ~12 bytes each the file is ~24 KB — fits in ESP8266 RAM.
 * ========================================================================= */

/**
 * @brief Append one reading; evict oldest if buffer is full.
 */
static void append_record(uint32_t counter, float temp)
{
    /* Load existing data */
    static float    temps[MAX_RECORDS];
    static uint32_t counters[MAX_RECORDS];
    int n = load_history(temps, counters, MAX_RECORDS);

    /* Evict oldest record when full (circular buffer behaviour) */
    if (n >= MAX_RECORDS) {
        /* Shift array left by 1: index 0 (oldest) is discarded */
        memmove(temps,    temps    + 1, (MAX_RECORDS - 1) * sizeof(float));
        memmove(counters, counters + 1, (MAX_RECORDS - 1) * sizeof(uint32_t));
        n = MAX_RECORDS - 1;
    }

    /* Append newest record at the end */
    counters[n] = counter;
    temps[n]    = temp;
    n++;

    /* Rewrite the whole file */
    FILE *f = fopen("/spiffs/log.csv", "w");
    if (!f) { ESP_LOGE(TAG, "Cannot open log.csv for writing"); return; }

    for (int i = 0; i < n; i++) {
        fprintf(f, "%lu,%.1f\n", (unsigned long)counters[i], temps[i]);
    }
    fclose(f);
    ESP_LOGI(TAG, "Log updated: %d records (oldest counter=%lu)",
             n, (unsigned long)counters[0]);
}

/**
 * @brief Load all records from log file into caller-supplied arrays.
 * @return Number of records loaded (0 if file doesn't exist yet).
 */
static int load_history(float *out_temps, uint32_t *out_counters, int max_records)
{
    FILE *f = fopen("/spiffs/log.csv", "r");
    if (!f) return 0;

    int   n    = 0;
    char  line[32];
    while (fgets(line, sizeof(line), f) && n < max_records) {
        unsigned long cnt;
        float         t;
        if (sscanf(line, "%lu,%f", &cnt, &t) == 2) {
            out_counters[n] = (uint32_t)cnt;
            out_temps[n]    = t;
            n++;
        }
    }
    fclose(f);
    return n;
}

/* =========================================================================
 * CORE ALGORITHM
 * =========================================================================
 *
 * GOAL: Tell the user when the hot water pipe is carrying genuinely hot
 * water worth using — without requiring any manual calibration.
 *
 * PROBLEM: "Hot" is relative. In Moscow buildings typical supply is 60°C;
 * in some Ukrainian buildings it may be 45°C; in summer after maintenance
 * it might be only 40°C. A fixed threshold would miss real use cases or
 * fire false positives.
 *
 * SOLUTION — adaptive percentile threshold:
 *
 *   1. Sort the historical temperature readings (7 days, 5-min intervals).
 *   2. Find the P-th percentile value (default P = 75).
 *      → This is the temperature that was exceeded 25% of the time.
 *   3. Declare the pipe "hot" if:
 *        current_temp ≥ percentile_threshold - HYSTERESIS_C
 *        AND current_temp ≥ ABSOLUTE_MIN_HOT_C
 *
 * INTERPRETATION:
 *   The percentile self-calibrates to the building's actual distribution.
 *   If the pipe is cold 80% of the time (common in Russian multi-family
 *   housing: hot water is cut for maintenance or follows a schedule), the
 *   75th percentile threshold will sit at the temperature the pipe reaches
 *   only during genuine hot-water delivery windows. We catch exactly those
 *   moments.
 *
 *   If hot water runs 24/7 (well-maintained building), the 75th percentile
 *   will be high (e.g. 60°C), so we only trigger on the truly hottest
 *   periods — still useful for "peak heat available" detection.
 *
 * HYSTERESIS:
 *   Prevents the output from flickering when the temperature oscillates
 *   right around the threshold. Once hot, it stays "hot" until it drops
 *   more than HYSTERESIS_C below the threshold.
 *
 * ABSOLUTE FLOOR:
 *   Even if the building had no hot water for 6 of the 7 days so the 75th
 *   percentile is 22°C, we still won't declare 23°C "hot". 40°C is the
 *   sanitary minimum for hot water supply in CIS countries.
 *
 * STARTUP:
 *   We require MIN_HISTORY_RECORDS (≥48, i.e. ≥4 hours) before the
 *   algorithm activates. This avoids the first-boot false positive where
 *   we have only 1 reading and 100% of history is "above the 75th pct".
 * ========================================================================= */

/** qsort comparator for float ascending sort */
static int cmp_float_asc(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

/**
 * @brief Evaluate whether current temperature is "historically hot".
 *
 * @param current_temp  Current DHT11 reading in °C
 * @param history       Array of past readings (any order; we sort internally)
 * @param history_len   Number of valid elements in history[]
 * @return true if pipe is currently in a "hot" state
 */
static bool evaluate_pipe_hot(float current_temp, float *history, int history_len)
{
    /* ---- Hard floor ---- */
    if (current_temp < ABSOLUTE_MIN_HOT_C) {
        ESP_LOGI(TAG, "Below floor (%.0f°C < %.0f°C)", current_temp, ABSOLUTE_MIN_HOT_C);
        return false;
    }

    /* ---- Copy + sort ---- */
    int   n      = (history_len < MAX_RECORDS) ? history_len : MAX_RECORDS;
    float *sorted = malloc(n * sizeof(float));
    if (!sorted) {
        ESP_LOGE(TAG, "malloc failed in evaluate");
        return false;
    }
    memcpy(sorted, history, n * sizeof(float));
    qsort(sorted, n, sizeof(float), cmp_float_asc);

    /* ---- Percentile by linear interpolation ----
     *
     * Index of the P-th percentile in a sorted array of length n:
     *   idx = (P/100) * (n-1)
     * e.g. P=75, n=100 → idx=74.25 → interpolate between sorted[74] and sorted[75]
     */
    float pidx  = (HOT_PERCENTILE / 100.0f) * (float)(n - 1);
    int   lo    = (int)pidx;
    float frac  = pidx - (float)lo;
    float threshold;

    if (lo + 1 < n) {
        threshold = sorted[lo] + frac * (sorted[lo + 1] - sorted[lo]);
    } else {
        threshold = sorted[lo];
    }
    free(sorted);

    ESP_LOGI(TAG, "P%.0f=%.1f°C | now=%.1f°C | hot if >=%.1f°C",
             HOT_PERCENTILE, threshold, current_temp,
             threshold - HYSTERESIS_C);

    return (current_temp >= threshold - HYSTERESIS_C);
}

/* =========================================================================
 * Output helpers
 * ========================================================================= */

static void led_on(void)  { gpio_set_level(PIN_LED, 1); }
static void led_off(void) { gpio_set_level(PIN_LED, 0); }

/**
 * @brief Drive buzzer for @times short beeps.
 * @param times   How many beeps
 * @param on_ms   Buzzer ON duration per beep (ms)
 * @param off_ms  Silence between beeps (ms)
 */
static void buzzer_beep(int times, int on_ms, int off_ms)
{
    for (int i = 0; i < times; i++) {
        gpio_set_level(PIN_BUZZER, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(PIN_BUZZER, 0);
        if (i < times - 1) {
            vTaskDelay(pdMS_TO_TICKS(off_ms));
        }
    }
}

/**
 * @brief Enter deep sleep for SLEEP_DURATION_US microseconds.
 *
 * HARDWARE NOTE: GPIO16 (D0 on NodeMCU) MUST be connected to the RST pin!
 * Without this wire, the RTC timer interrupt has nowhere to go and the
 * chip will sleep forever (until battery dies or manual reset).
 *
 * After wake the chip performs a full reset and app_main() runs again.
 * There is no "resume from sleep" — deep sleep is essentially a very
 * low-power reboot timer.
 */
static void go_to_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep for 5 min. GPIO16→RST required!");
    /* Let UART transmit buffer flush */
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_deep_sleep(SLEEP_DURATION_US);
    /* never reached */
}
