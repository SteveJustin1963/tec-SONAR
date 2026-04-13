// ============================================================
//  EchoCloud_Nano – Main Sketch
//  Multi-Receiver Acoustic Point-Cloud Sonar for Arduino Nano
//  Board: Arduino Nano (ATmega328P old/new bootloader)
//  FQBN : arduino:avr:nano:cpu=atmega328old  (clone boards)
//         arduino:avr:nano:cpu=atmega328      (official Nano)
//  v1.0  2026-04-13
//
//  ── KEY NANO vs ESP32 DIFFERENCES ──────────────────────────
//  Feature          ESP32               Nano
//  ─────────────────────────────────────────────────────────
//  CPU speed        240 MHz             16 MHz
//  SRAM             520 KB              2 KB  ← tight!
//  ADC resolution   12-bit (0–4095)     10-bit (0–1023)
//  ADC stored as    uint16_t            uint8_t (top 8 bits)
//  Buffer size      600 samples         200 samples
//  40 kHz gen.      LEDC peripheral     Timer1 hardware PWM
//  TX pin           GPIO26 (any)        Pin 9  (OC1A fixed)
//  Float printf     Serial.printf()     dtostrf() + Serial
//  Voltage logic    3.3 V               5 V  (simpler circuit)
//  Voltage divider  Required on RX      NOT needed (5 V ADC)
//  Max range        ~15 m               ~12 m (slower ADC)
//  Update rate      5 Hz                4 Hz
//  ─────────────────────────────────────────────────────────
//
//  ASCII FLOWCHART — NANO MAIN LOOP
//
//   setup()
//     │
//     ├─ Serial.begin(115200)
//     ├─ sonar_nano_init()
//     │     ├─ Configure Timer1: Fast PWM, ICR1=399, f=40kHz
//     │     ├─ pinMode(9, OUTPUT) — TX pin (OC1A)
//     │     └─ ADC auto-configured by analogRead()
//     └─ print CSV header
//
//   loop() ──────────────────────────────────────────────────
//     │
//     ▼
//     ┌──────────────────────────────────────────────────┐
//     │  sonar_nano_ping_and_sample(buf)                 │
//     │                                                  │
//     │  TCCR1B |= CS10   ← start Timer1 (40 kHz on 9)  │
//     │  delayMicroseconds(500)  ← 20 cycles             │
//     │  TCCR1B &= ~CS1x  ← stop Timer1                 │
//     │  digitalWrite(9, LOW)                            │
//     │                                                  │
//     │  t_start = micros()                              │
//     │  for s in 0..199:                                │
//     │    for ch in 0..3:                               │
//     │      buf[ch][s] = analogRead(A0+ch) >> 2        │
//     │  sample_period = elapsed / 200                   │
//     └──────────────────────────────────────────────────┘
//     │
//     ▼
//     ┌──────────────────────────────────────────────────┐
//     │  sonar_nano_process(buf, result)                 │
//     │                                                  │
//     │  for ch in 0..3:                                 │
//     │    noise = mean(buf[ch][0..14])  (dead zone)    │
//     │    thresh = noise × 3                            │
//     │    pulse_len = 500µs / sample_period_us          │
//     │                                                  │
//     │    sliding-window sum (width=pulse_len):         │
//     │      find max sum and its index                  │
//     │      if max_sum > thresh × pulse_len:            │
//     │        echo_idx = centre of best window          │
//     │        distance = echo_idx/rate × 343/2          │
//     │      else:  distance = -1.0 (no echo)            │
//     └──────────────────────────────────────────────────┘
//     │
//     ▼
//     ┌──────────────────────────────────────────────────┐
//     │  sonar_nano_serial_output(result)                │
//     │  → CSV: millis(),d0,d1,d2,d3,mask               │
//     │                                                  │
//     │  sonar_nano_to_points() → points[]               │
//     │  sonar_nano_print_points() → "POINT,x,y"        │
//     └──────────────────────────────────────────────────┘
//     │
//     ▼
//     delay(250ms)
//     └──────────────────────────────────────────────────► (repeat)
//
// ============================================================
#include <Arduino.h>
#include "config_nano.h"
#include "sonar_nano.h"

// ── Global buffers ───────────────────────────────────────────
// Declared globally to avoid blowing the 2 KB stack.
static SonarBufferNano buf;
static SonarResultNano result;
static Point2DNano     points[NUM_RX_NANO];

// ── setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD_NANO);
    // Short delay — USB-serial CDC enumeration on genuine Nano
    delay(300);

    Serial.println(F("#"));
    Serial.println(F("# EchoCloud_Nano v1.0"));
    Serial.println(F("# Arduino Nano / ATmega328P"));
    Serial.println(F("#"));

    sonar_nano_init();

#if DEBUG_RAM_NANO
    Serial.print(F("# Free RAM after init: "));
    Serial.print(sonar_nano_free_ram());
    Serial.println(F(" bytes"));
#endif

#if OUTPUT_CSV_NANO
    Serial.println(F("timestamp_ms,rx0_m,rx1_m,rx2_m,rx3_m,valid_mask"));
#endif

    Serial.println(F("# READY"));
}

// ── loop ─────────────────────────────────────────────────────
void loop() {
    // Step 1: Fire ping, capture echoes
    sonar_nano_ping_and_sample(buf);

    // Step 2: Cross-correlate → distances
    sonar_nano_process(buf, result);

    // Step 3: Output distances
    sonar_nano_serial_output(result);

    // Step 4: Build and output 2D point cloud
    uint8_t n = sonar_nano_to_points(result, points, NUM_RX_NANO);
    sonar_nano_print_points(points, n);

    // Step 5: Wait (prevents echo overlap between pings)
    delay(SCAN_INTERVAL_NANO);
}
