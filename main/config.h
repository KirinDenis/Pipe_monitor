#pragma once

/**
 * @file config.h
 * @brief Hardware pins and algorithm constants.
 *        All tuning is done here — no need to touch main.c.
 *
 * ESP32 DevKit pinout used below.
 * Change GPIO numbers freely — any output-capable GPIO works for LED/buzzer,
 * any input-capable GPIO works for DHT11.
 * Avoid: GPIO 6-11 (connected to internal flash), GPIO 34-39 (input only).
 */

/* -------------------------------------------------------------------------
 * GPIO assignments
 *
 *  DHT11 data line needs a 4.7 kΩ pull-up resistor to 3.3V.
 *  LED:    anode → 330 Ω → GPIO, cathode → GND
 *  Buzzer: use an ACTIVE buzzer (buzzes at DC); connect + to GPIO, - to GND.
 *          If buzzer draws >40 mA add a NPN transistor (e.g. 2N2222).
 *
 *  DEEP SLEEP WAKE: ESP32 uses any RTC-capable GPIO as wakeup source
 *  via timer — no external GPIO→RST wire needed (unlike ESP8266)!
 * ------------------------------------------------------------------------- */
#define PIN_DHT      GPIO_NUM_4    /* D4 - DHT11 data                       */
#define PIN_LED      GPIO_NUM_2    /* D2 - Red LED (built-in LED on DevKit)  */
#define PIN_BUZZER   GPIO_NUM_15   /* D15 - Active buzzer                    */

/* -------------------------------------------------------------------------
 * DHT11 timing
 *
 * DHT11 needs 1 second to stabilise after power-on before first read.
 * After deep sleep ESP32 fully reboots, so we always wait.
 * 1500 ms gives comfortable margin.
 * ------------------------------------------------------------------------- */
#define DHT_STABILISE_MS    1500

/* -------------------------------------------------------------------------
 * Deep sleep duration: 5 minutes
 * ------------------------------------------------------------------------- */
#define SLEEP_DURATION_US   (5ULL * 60ULL * 1000000ULL)

/* -------------------------------------------------------------------------
 * Log / history settings
 *
 * 5 min interval × 12/hour × 24 h × 7 days = 2016 records per week.
 * Each CSV line ≈ 14 bytes → 2016 × 14 ≈ 28 KB — fits easily in SPIFFS.
 *
 * MIN_HISTORY_RECORDS: algorithm is silent until we have this many readings.
 * 48 records = 4 hours of baseline — enough to see at least one hot/cold cycle.
 * ------------------------------------------------------------------------- */
#define MAX_RECORDS             2016
#define MIN_HISTORY_RECORDS       48

/* -------------------------------------------------------------------------
 * Algorithm thresholds
 *
 * HOT_PERCENTILE:
 *   Current temperature must exceed this percentile of all past readings.
 *   75 → "hotter than 75% of what we've seen" → top quarter of history.
 *   Raise to 80-85 for stricter detection, lower to 65-70 for looser.
 *
 * ABSOLUTE_MIN_HOT_C:
 *   Hard floor regardless of percentile. CIS sanitary norms require
 *   hot water supply ≥ 60°C at source (≥ 40°C at tap). Set to taste.
 *
 * HYSTERESIS_C:
 *   Tolerance below threshold to prevent flickering when temperature
 *   is right on the boundary (e.g. threshold=50°C, triggers at 49°C too).
 * ------------------------------------------------------------------------- */
#define HOT_PERCENTILE          75.0f   /* percentile (0–100)               */
#define ABSOLUTE_MIN_HOT_C      40.0f   /* degrees Celsius                  */
#define HYSTERESIS_C             1.0f   /* degrees Celsius                  */

/* -------------------------------------------------------------------------
 * LED / buzzer signal parameters
 * ------------------------------------------------------------------------- */
#define BUZZER_BEEP_COUNT    5      /* beeps when pipe becomes hot           */
#define BUZZER_ON_MS       120      /* beep duration, ms                     */
#define BUZZER_OFF_MS      180      /* silence between beeps, ms             */
#define LED_VISIBLE_MS    3000      /* keep LED on after buzzer sequence, ms */
