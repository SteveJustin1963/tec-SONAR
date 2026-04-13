// ============================================================
//  EchoCloud – Sonar Module Header
//  Multi-Receiver Acoustic Point-Cloud Sonar for ESP32
// ============================================================
#pragma once
#include <Arduino.h>
#include "config.h"

// ── Data Structures ──────────────────────────────────────────

/**
 * Raw ADC envelope buffer for one complete ping event.
 * Each row is one receiver channel; columns are samples in time.
 * Storing as uint16_t saves RAM (4-byte float not needed here).
 */
struct SonarBuffer {
    uint16_t data[NUM_RX][MAX_SAMPLES];  // Raw ADC readings
    uint32_t num_samples;                // Actual samples captured
    uint32_t sample_period_us;           // Measured µs per sample
};

/**
 * Processed result for one ping.
 * distances[]  : metres from each receiver to closest reflector
 * valid[]      : true if echo was clearly detected
 */
struct SonarResult {
    float    distances[NUM_RX];
    bool     valid[NUM_RX];
    float    sample_rate_hz;
};

/**
 * 2D point in robot-centric coordinates (metres).
 */
struct Point2D {
    float x;   // +x = forward
    float y;   // +y = left
};

// ── Public API ────────────────────────────────────────────────

/**
 * Initialise LEDC transmitter, ADC pins, and (optional) LED.
 * Call once in setup().
 */
void sonar_init(void);

/**
 * Fire one ultrasonic burst, then capture all receiver channels
 * as fast as possible into buf.
 * Blocking: returns when MAX_SAMPLES collected or window expires.
 */
void sonar_ping_and_sample(SonarBuffer &buf);

/**
 * Process a raw buffer into distances.
 * Uses a sliding-window cross-correlation with the expected
 * pulse shape (rectangular, width = burst duration in samples).
 * Skips the dead zone to avoid direct-blast false positives.
 */
void sonar_process(const SonarBuffer &buf, SonarResult &result);

/**
 * Convert distances + known receiver positions into 2D map points.
 * Simple centroid method for closely-spaced receivers:
 *   point = bearing from receiver position + distance.
 * More advanced TDOA triangulation available via sonar_tdoa().
 */
uint8_t sonar_to_points(const SonarResult &result, Point2D *points, uint8_t max_points);

/**
 * Optional TDOA triangulation between receiver pairs.
 * Returns one (x,y) point from three or more valid receivers.
 * Requires at least 3 valid echoes.
 */
bool sonar_tdoa(const SonarResult &result, Point2D &point);

/**
 * Print SonarResult in CSV or JSON format over Serial.
 * Format (CSV):  timestamp_ms,rx0_m,rx1_m,rx2_m,rx3_m,valid_mask
 * Format (JSON): {"t":1234,"rx":[1.2,0.8,1.5,0.9],"v":15}
 */
void sonar_serial_output(const SonarResult &result);

/**
 * Print a 2D point cloud over Serial.
 * Format:  POINT,x,y  (one line per point)
 */
void sonar_print_points(const Point2D *points, uint8_t count);

// ── Internal helpers (exposed for unit-testing) ────────────────

/**
 * Find the sample index of the strongest echo in one channel.
 * Returns -1 if no echo exceeds threshold.
 *
 * Algorithm: sliding-window sum (cross-correlation with rect pulse)
 * Window width = pulse_samples.
 * Threshold = noise_floor * THRESHOLD_MULTIPLIER.
 */
int16_t sonar_find_echo(const uint16_t *channel, uint32_t num_samples,
                         uint16_t pulse_samples, uint16_t threshold);

/**
 * Compute mean of first N samples (used as noise floor estimate).
 */
uint16_t sonar_noise_floor(const uint16_t *channel, uint16_t n);
