// ============================================================
//  EchoCloud – Configuration
//  Multi-Receiver Acoustic Point-Cloud Sonar for ESP32
//  v1.0  2026-04-13
// ============================================================
#pragma once

// ── ESP32 Pin Assignments ────────────────────────────────────
//  Transmitter: driven via NPN transistor buffer (see README)
#define TX_PIN          26      // GPIO26 → transistor base (100Ω series)

//  Receivers: connect to envelope-detector outputs
//  Must be ADC1 channels only (ADC2 is shared with WiFi)
//  GPIO36/39/34/35 are input-only — perfect for ADC
#define RX_PIN_0        36      // ADC1_CH0 — Receiver 0 (FRONT-LEFT)
#define RX_PIN_1        39      // ADC1_CH3 — Receiver 1 (FRONT-RIGHT)
#define RX_PIN_2        34      // ADC1_CH6 — Receiver 2 (REAR-LEFT)
#define RX_PIN_3        35      // ADC1_CH7 — Receiver 3 (REAR-RIGHT)

//  Optional servo for rotating head scan
#define SERVO_PIN       18      // GPIO18 — Servo signal
#define SERVO_ENABLED   0       // Set to 1 to enable servo scanning

//  Optional status LED
#define LED_PIN         2       // Built-in LED on most ESP32 DevKit boards
#define LED_ENABLED     1

// ── Sonar Parameters ─────────────────────────────────────────
#define NUM_RX          4       // Number of receiver channels

#define TX_FREQ_HZ      40000   // Ultrasonic frequency  (Hz)
#define TX_BURST_US     500     // Burst duration        (µs) = 20 cycles @ 40kHz
#define TX_LEDC_CHANNEL 0       // ESP32 LEDC channel for transmitter
#define TX_LEDC_RES     8       // LEDC resolution bits  (8-bit = 0..255)
#define TX_DUTY         128     // 50 % duty cycle       (128 / 255)

// ADC / Sampling
//  analogRead() on ESP32 takes ~20-50 µs per call.
//  With 4 receivers × 4 readings averaged = ~16 calls per sample set.
//  Effective sample rate ≈ 4,000-5,000 samples/s per channel.
//  At 5 ksps: range resolution = 343/(2×5000) ≈ 3.4 cm  ✓
//  Max range = 20 m → round-trip = 116 ms → 580 samples  ✓
#define ADC_OVERSAMPLE  4       // Readings averaged per sample (noise reduction)
#define MAX_SAMPLES     600     // Samples to capture per ping (≈120 ms window)
#define DEAD_ZONE_SAMPLES 20    // Skip first N samples (direct blast / ringing)

#define ADC_RESOLUTION  12      // ESP32 ADC bits (12-bit = 0..4095)
#define ADC_ATTEN       ADC_11db // Input attenuation: 0-3.3 V range

// ── Geometry ─────────────────────────────────────────────────
//  Receiver positions on the robot body (x, y) in metres
//  Origin = robot centre.  +x = forward,  +y = left.
//  Default: 4 receivers in a 10 cm square arrangement.
#define RX_POS_X  { 0.05f,  0.05f, -0.05f, -0.05f }  // metres
#define RX_POS_Y  { 0.05f, -0.05f,  0.05f, -0.05f }  // metres

#define SPEED_OF_SOUND  343.0f  // m/s  (≈20 °C, sea level)
// Correction formula (if you have a DHT11/22 temperature sensor):
// c = 331.3f + 0.606f * temperature_celsius

// ── Output / Protocol ────────────────────────────────────────
#define SERIAL_BAUD     115200
#define SCAN_INTERVAL_MS 200    // Minimum ms between pings
#define OUTPUT_CSV      1       // 1 = CSV,  0 = JSON

// ── Servo Scan Parameters (only used if SERVO_ENABLED = 1) ───
#define SERVO_MIN_DEG   0
#define SERVO_MAX_DEG   180
#define SERVO_STEP_DEG  5
#define SERVO_SETTLE_MS 30      // Wait after each servo move

// ── Debug ─────────────────────────────────────────────────────
#define DEBUG_RAW       0       // 1 = dump raw ADC buffer over serial
#define DEBUG_TIMING    0       // 1 = print sample-rate measurement
