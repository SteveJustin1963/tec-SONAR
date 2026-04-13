// ============================================================
//  EchoCloud_Nano – Sonar Module Header
//  Arduino Nano / ATmega328P version
// ============================================================
#pragma once
#include <Arduino.h>
#include "config_nano.h"

// ── Data Structures ──────────────────────────────────────────

/**
 * Raw 8-bit envelope buffer for one complete ping.
 * Stores top 8 bits of the 10-bit ADC reading (>> 2).
 * uint8_t halves RAM usage vs uint16_t: critical on 2 KB SRAM.
 */
struct SonarBufferNano {
    uint8_t  data[NUM_RX_NANO][MAX_SAMPLES_NANO]; // Raw ADC >> 2
    uint16_t num_samples;                          // Samples captured
    uint16_t sample_period_us;                     // Measured µs/sample
};

/**
 * Processed result for one ping.
 */
struct SonarResultNano {
    float   distances[NUM_RX_NANO];   // metres,  −1.0 = no echo
    bool    valid[NUM_RX_NANO];
    float   sample_rate_hz;
};

/**
 * 2D point in robot-centric coordinates (metres).
 */
struct Point2DNano {
    float x;
    float y;
};

// ── Public API ────────────────────────────────────────────────

/**
 * Initialise Timer1 for 40 kHz TX, configure ADC pins, LED.
 * Call once in setup().
 */
void sonar_nano_init(void);

/**
 * Fire one 500 µs burst via Timer1 OC1A (pin 9),
 * then capture all receiver channels into buf.
 * Blocking.
 */
void sonar_nano_ping_and_sample(SonarBufferNano &buf);

/**
 * Find echo peak in one 8-bit channel using a sliding window.
 * Returns sample index of echo centre, or -1 if below threshold.
 */
int16_t sonar_nano_find_echo(const uint8_t *channel,
                              uint16_t num_samples,
                              uint8_t  pulse_samples,
                              uint8_t  threshold);

/**
 * Estimate noise floor as mean of the first n samples.
 */
uint8_t sonar_nano_noise_floor(const uint8_t *channel, uint8_t n);

/**
 * Process raw buffer → distances.
 */
void sonar_nano_process(const SonarBufferNano &buf,
                         SonarResultNano &result);

/**
 * Convert distances + receiver positions → 2D point cloud.
 * Returns number of valid points written into points[].
 */
uint8_t sonar_nano_to_points(const SonarResultNano &result,
                               Point2DNano *points,
                               uint8_t max_points);

/**
 * Output distances as CSV or JSON over Serial.
 * CSV: timestamp_ms,rx0_m,rx1_m,rx2_m,rx3_m,valid_mask
 */
void sonar_nano_serial_output(const SonarResultNano &result);

/**
 * Print 2D points: "POINT,x,y"
 */
void sonar_nano_print_points(const Point2DNano *points, uint8_t count);

// ── RAM diagnostic ────────────────────────────────────────────
/** Returns number of free SRAM bytes. Uses stack/heap gap method. */
int sonar_nano_free_ram(void);
