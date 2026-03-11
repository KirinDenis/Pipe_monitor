# Pipe Temperature Monitor — ESP32 / ESP-IDF v5.5

Monitors a hot-water supply pipe, stores 7 days of temperature history,
and alerts (LED + buzzer) when the pipe is carrying genuinely hot water.
Detection is adaptive — no manual calibration needed.

<img width="50%" height="50%" alt="image" src="[https://github.com/KirinDenis/ImageClusterizer/blob/main/Screen/logo.png](https://github.com/KirinDenis/Pipe_monitor/blob/main/image1.jpg)" />

---

## Quick Start

```bash
git clone / copy project folder
cd pipe_monitor

idf.py set-target esp32
idf.py build
idf.py -p COM3 flash monitor     # Windows: COM3, Linux: /dev/ttyUSB0
```

### Please note that for a file partitions_custom.csv 
with custom partitions, do not rename it to partitions.csv(used by default).
Also, ESP-IDF can independently change the original sdkconfig file -> pay attention to these keys section:

CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions_custom.csv"
CONFIG_PARTITION_TABLE_FILENAME="partitions_custom.csv"
CONFIG_PARTITION_TABLE_OFFSET=0x8000
CONFIG_PARTITION_TABLE_MD5=y

they keys should be like this. In any case, this doesn't work for me if this file is named partitions.csv for ESP-IDF v5.5.
you see this error at "Monitor Device":
E (409) SPIFFS: spiffs partition could not be found
E (409) pipe_monitor: SPIFFS: ESP_ERR_NOT_FOUND
E (409) pipe_monitor: SPIFFS failed — going back to sleep

---

## Wiring

```
ESP32 DevKit
                    3.3V
                     │
                 [4.7 kΩ]         ← pull-up REQUIRED for DHT11
                     │
DHT11 DATA ──────────┤──── GPIO4
DHT11 VCC  ──────────────── 3.3V
DHT11 GND  ──────────────── GND

GPIO2  ──[330 Ω]──>|── GND        Red LED (anode to GPIO)
                                  GPIO2 = built-in LED on most DevKits

GPIO15 ──────────── Buzzer +      Active buzzer (buzzes at DC 3.3V)
                    Buzzer − ──── GND

9V "Krona" :) → LDO IN
LDO 3.3V      → ESP32 VIN, DHT11 VCC
LDO GND       → GND

Unlike ESP8266, ESP32 does NOT need GPIO16→RST for deep sleep!!!
The RTC timer wakes the chip internally.
```

### LDO choice matters for battery life!

| LDO | Quiescent current | Notes |
|-----|-------------------|-------|
| AMS1117-3.3 | ~5 mA | Common on DevKit boards; drains Krona in ~4 days |
| MCP1700-3302 | 2 µA | Best choice for this project; ~25+ days |
| XC6206P332 | 1 µA | Also excellent |

If using a full DevKit board (not bare ESP32 module), the onboard AMS1117
and USB-UART chip add ~10–20 mA quiescent draw. For long battery life,
use a bare ESP32-WROOM module with MCP1700.

or 

Bypass the regulator completely

ESP32 DevKit or Arduino DevBoard and a 3.3V pin
You can:
 - DO NOT supply power to VIN
 - DO NOT use USB
 - Supply a stable 3.3V directly to the 3.3V pin

Then the AMS1117 is not involved.
This is the most convenient method.

---

## Project Structure

```
pipe_monitor/
├── CMakeLists.txt              ← top-level (required by IDF)
├── partitions_custom.csv       ← custom partition table with SPIFFS
├── sdkconfig.defaults          ← CPU=80MHz, WiFi off, power management
├── main/
    ├── CMakeLists.txt          ← registers main.c with build system
    ├── config.h                ← ALL tunable pins and constants
    ├── main.c                  ← application logic
    ├── dht.h
    └── dht.c                   ← DHT11/DHT22 bit-bang driver
```

---

## Algorithm

### The problem with fixed thresholds

If you hardcode `if (temp > 50°C) → hot`, this breaks across buildings:
- Building A: hot water is 45°C → never triggers
- Building B: hot water is 65°C → triggers even when pipe is lukewarm
- Building with scheduled supply: cold 80% of day, hot 20% → threshold unclear

### Adaptive percentile threshold

Every 5 minutes:
1. Read temperature, append to log (7-day circular buffer, 2016 entries max)
2. Sort all historical readings
3. Compute the **75th percentile** (P75) — temperature exceeded by top 25% of readings
4. Declare pipe **hot** if:
   - `current_temp ≥ P75 − 1°C` (1°C hysteresis to avoid flickering)
   - **AND** `current_temp ≥ 40°C` (absolute sanitary minimum floor)

### Why this works

In a building where hot water runs 3 hours/day, the temperature distribution
is bimodal: ~20°C most of the day, ~55°C during delivery. The 75th percentile
naturally falls at the beginning of the hot cluster — the algorithm fires
exactly when hot water arrives, regardless of the absolute temperature.

### Tuning (edit `config.h`)

| Constant | Default | Effect if raised |
|---|---|---|
| `HOT_PERCENTILE` | 75.0 | Stricter — only very hot triggers |
| `ABSOLUTE_MIN_HOT_C` | 40.0°C | Won't fire below this ever |
| `HYSTERESIS_C` | 1.0°C | Reduces boundary flicker |
| `MIN_HISTORY_RECORDS` | 48 (4 h) | Longer baseline before algorithm activates |

---

## Log File Format

`/spiffs/log.csv` — readable via serial + SPIFFS tools:
```
1001,23.0
1002,23.0
1003,54.0
...
```
`counter` field: monotonic wake counter. Survives resets — gaps in counter
values indicate unexpected reboots. Time axis: `T = deploy_epoch + counter × 300s`

`/spiffs/counter.txt` — current counter value (plain text, editable).

---

## Power Consumption Summary

| State | Current | Duration |
|---|---|---|
| Deep sleep (ESP32 RTC only) | ~10 µA | ~298 s per cycle |
| Active (CPU + DHT + SPIFFS) | ~80 mA | ~3 s per cycle |
| Buzzer beeping | +30 mA | ~1.5 s total |
| **Average (MCP1700 LDO)** | **~0.82 mA** | |
| Krona 500 mAh estimated life | **~600 h / 25 days** | |
