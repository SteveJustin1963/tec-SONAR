// ============================================================
//  EchoCloud_Nano – Configuration
//  Multi-Receiver Acoustic Point-Cloud Sonar for Arduino Nano
//  ATmega328P @ 16 MHz,  2 KB SRAM,  32 KB Flash,  5 V logic
//  v1.0  2026-04-13
//
//  KEY DIFFERENCE FROM ESP32 VERSION
//  ──────────────────────────────────────────────────────────
//  ATmega328P has only 2 KB SRAM.
//  Buffer: NUM_RX × MAX_SAMPLES × 1 byte (uint8_t, 8-bit)
//  = 4 × 200 × 1 = 800 bytes  — fits with headroom.
//
//  ADC is 10-bit (0–1023) but we store only the top 8 bits
//  (>> 2) to halve RAM usage; resolution loss is negligible
//  because envelope SNR limits accuracy far more than 2 bits.
//
//  40 kHz generation uses Timer1 hardware PWM on pin 9 (OC1A).
//  No LEDC peripheral (ESP32 feature); no analogSetAttenuation.
//  ──────────────────────────────────────────────────────────
// ============================================================
#pragma once
#include <Arduino.h>

// ── Pin Assignments ──────────────────────────────────────────
//  TX: Timer1 OC1A is hardware-wired to pin 9 on the Nano.
#define TX_PIN_NANO     9       // PWM output → 2N2222 base

//  RX: analog pins A0–A3
#define RX_PIN_0        A0      // Receiver 0 (FRONT-LEFT)
#define RX_PIN_1        A1      // Receiver 1 (FRONT-RIGHT)
#define RX_PIN_2        A2      // Receiver 2 (REAR-LEFT)
#define RX_PIN_3        A3      // Receiver 3 (REAR-RIGHT)

#define LED_PIN_NANO    13      // Built-in LED

// ── Sonar Parameters ─────────────────────────────────────────
#define NUM_RX_NANO     4       // Receivers to read

//  40 kHz from Timer1 Fast PWM, ICR1 as TOP, no prescaler:
//    f = F_CPU / (ICR1 + 1)  →  ICR1 = 16,000,000/40,000 − 1 = 399
#define TIMER1_ICR1     399U    // TOP register value  (→ exactly 40 kHz)
#define TIMER1_OCR1A    200U    // Compare = ICR1/2   (50 % duty cycle)

#define TX_BURST_US     500     // Burst duration in µs  (20 cycles @ 40 kHz)

//  RAM budget:  2048 bytes total SRAM on ATmega328P
//    Buffer:    4 × 200 × 1 = 800 bytes
//    Overhead:  ~400 bytes (stack, framework, locals)
//    Free:      ~848 bytes  ✓
//
//  analogRead on Nano ≈ 112 µs per call (default prescaler 128)
//  4 channels × 112 µs = 448 µs per sample set
//  Effective rate per channel ≈ 2232 sps
//  200 samples × (1/2232 s) × 343/2 ≈ 15.4 m max range  ✓
//  Range resolution = 343 × 448 µs / 2 ≈ 7.7 cm

#define MAX_SAMPLES_NANO    200     // ADC samples per ping (fits 2 KB RAM)
#define DEAD_ZONE_NANO      15      // Skip first N samples (direct blast)
#define THRESHOLD_MULT_NANO 3       // Echo must be > floor × this value
#define SPEED_OF_SOUND_F    343.0f  // m/s

// ── Geometry ─────────────────────────────────────────────────
//  Same physical layout as ESP32 version: 4 receivers in a
//  10 cm square centred on the robot.
#define RX_POS_X_NANO  { 0.05f,  0.05f, -0.05f, -0.05f }
#define RX_POS_Y_NANO  { 0.05f, -0.05f,  0.05f, -0.05f }

// ── Output ────────────────────────────────────────────────────
#define SERIAL_BAUD_NANO    115200
#define SCAN_INTERVAL_NANO  250     // ms between pings (slightly longer — slower CPU)
#define OUTPUT_CSV_NANO     1       // 1 = CSV,  0 = JSON

// ── Debug ─────────────────────────────────────────────────────
#define DEBUG_RAM_NANO      0       // 1 = print free RAM each loop
#define DEBUG_TIMING_NANO   0       // 1 = print measured sample rate
