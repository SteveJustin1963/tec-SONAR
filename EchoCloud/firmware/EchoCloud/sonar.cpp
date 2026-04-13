// ============================================================
//  EchoCloud – Sonar Module Implementation
//  Multi-Receiver Acoustic Point-Cloud Sonar for ESP32
// ============================================================
#include "sonar.h"
#include <math.h>

// ── Receiver pin table (built from config.h macros) ──────────
static const uint8_t RX_PINS[NUM_RX] = {
    RX_PIN_0, RX_PIN_1, RX_PIN_2, RX_PIN_3
};

// Receiver physical positions (metres, robot-centric)
static const float RX_X[NUM_RX] = RX_POS_X;
static const float RX_Y[NUM_RX] = RX_POS_Y;

// ── Threshold multiplier above noise floor ───────────────────
// Echo must be > noise_floor * this value to count as valid
#define THRESHOLD_MULT 3

// ── sonar_init ───────────────────────────────────────────────
void sonar_init(void) {
    // Transmitter: LEDC PWM at 40 kHz
    ledcAttach(TX_PIN, TX_FREQ_HZ, TX_LEDC_RES);
    ledcWrite(TX_PIN, 0);   // Start silent

    // Receiver ADC pins
    analogReadResolution(ADC_RESOLUTION);
    analogSetAttenuation(ADC_ATTEN);
    for (uint8_t i = 0; i < NUM_RX; i++) {
        pinMode(RX_PINS[i], INPUT);
    }

#if LED_ENABLED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
#endif

    Serial.println(F("# EchoCloud v1.0 — sonar ready"));
    Serial.printf("# TX=%d Hz  Burst=%d µs  Channels=%d\n",
                  TX_FREQ_HZ, TX_BURST_US, NUM_RX);
}

// ── sonar_ping_and_sample ────────────────────────────────────
void sonar_ping_and_sample(SonarBuffer &buf) {
    // ── 1. Transmit burst ───────────────────────────────────
#if LED_ENABLED
    digitalWrite(LED_PIN, HIGH);
#endif
    ledcWrite(TX_PIN, TX_DUTY);         // Start 40 kHz PWM
    delayMicroseconds(TX_BURST_US);     // Burst duration
    ledcWrite(TX_PIN, 0);               // Stop transmitter
#if LED_ENABLED
    digitalWrite(LED_PIN, LOW);
#endif

    // ── 2. Sample all receivers as fast as possible ─────────
    // We read each receiver ADC_OVERSAMPLE times and average
    // to reduce ADC noise (esp32 ADC can be noisy ±30 LSB).
    uint32_t t_start = micros();
    for (uint32_t s = 0; s < MAX_SAMPLES; s++) {
        for (uint8_t ch = 0; ch < NUM_RX; ch++) {
            uint32_t acc = 0;
            for (uint8_t ov = 0; ov < ADC_OVERSAMPLE; ov++) {
                acc += analogRead(RX_PINS[ch]);
            }
            buf.data[ch][s] = (uint16_t)(acc / ADC_OVERSAMPLE);
        }
    }
    uint32_t t_end = micros();

    buf.num_samples    = MAX_SAMPLES;
    buf.sample_period_us = (t_end - t_start) / MAX_SAMPLES;

#if DEBUG_TIMING
    Serial.printf("# Sample period = %lu µs  (~%.0f sps)\n",
                  buf.sample_period_us,
                  1e6f / (float)buf.sample_period_us);
#endif

#if DEBUG_RAW
    Serial.println(F("# RAW_START"));
    for (uint32_t s = 0; s < buf.num_samples; s++) {
        for (uint8_t ch = 0; ch < NUM_RX; ch++) {
            Serial.print(buf.data[ch][s]);
            if (ch < NUM_RX - 1) Serial.print(',');
        }
        Serial.println();
    }
    Serial.println(F("# RAW_END"));
#endif
}

// ── sonar_noise_floor ────────────────────────────────────────
uint16_t sonar_noise_floor(const uint16_t *channel, uint16_t n) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < n; i++) {
        sum += channel[i];
    }
    return (uint16_t)(sum / n);
}

// ── sonar_find_echo ──────────────────────────────────────────
// Sliding-window cross-correlation with a rectangular pulse.
// This is equivalent to a moving average, which maximises at
// the sample where the echo burst is centred.
// Returns the centre-sample index of the echo, or -1 if none.
int16_t sonar_find_echo(const uint16_t *channel, uint32_t num_samples,
                         uint16_t pulse_samples, uint16_t threshold) {
    if (num_samples < (uint32_t)(DEAD_ZONE_SAMPLES + pulse_samples)) {
        return -1;
    }

    // Start search after dead zone
    uint32_t search_start = DEAD_ZONE_SAMPLES;
    uint32_t search_end   = num_samples - pulse_samples;

    // Build first window sum
    uint32_t win_sum = 0;
    for (uint16_t k = 0; k < pulse_samples; k++) {
        win_sum += channel[search_start + k];
    }

    uint32_t best_sum = 0;
    uint32_t best_idx = 0;

    // Slide window
    for (uint32_t lag = search_start; lag < search_end; lag++) {
        if (win_sum > best_sum) {
            best_sum = win_sum;
            best_idx = lag;
        }
        // Slide: remove oldest, add newest
        win_sum -= channel[lag];
        win_sum += channel[lag + pulse_samples];
    }

    // Check if echo exceeds threshold
    uint32_t threshold_sum = (uint32_t)threshold * pulse_samples;
    if (best_sum < threshold_sum) {
        return -1;  // No valid echo
    }

    // Return centre of the window
    return (int16_t)(best_idx + pulse_samples / 2);
}

// ── sonar_process ────────────────────────────────────────────
void sonar_process(const SonarBuffer &buf, SonarResult &result) {
    result.sample_rate_hz = (buf.sample_period_us > 0)
        ? 1e6f / (float)buf.sample_period_us
        : 5000.0f;  // Fallback estimate

    // Pulse width in samples (how many samples wide is the burst echo?)
    uint16_t pulse_samples = (uint16_t)(
        (float)TX_BURST_US / (float)buf.sample_period_us
    );
    if (pulse_samples < 1) pulse_samples = 1;

    for (uint8_t ch = 0; ch < NUM_RX; ch++) {
        // Estimate noise from the dead-zone samples (before echo arrives)
        uint16_t noise = sonar_noise_floor(buf.data[ch], DEAD_ZONE_SAMPLES);
        uint16_t threshold = noise * THRESHOLD_MULT;

        // Find echo peak
        int16_t echo_sample = sonar_find_echo(
            buf.data[ch], buf.num_samples, pulse_samples, threshold
        );

        if (echo_sample < 0) {
            result.distances[ch] = -1.0f;   // No echo detected
            result.valid[ch]     = false;
        } else {
            // Convert sample index to round-trip time, then distance
            float round_trip_s = (float)echo_sample / result.sample_rate_hz;
            result.distances[ch] = (round_trip_s * SPEED_OF_SOUND) / 2.0f;
            result.valid[ch]     = (result.distances[ch] > 0.05f &&
                                    result.distances[ch] < 20.0f);
        }
    }
}

// ── sonar_to_points ──────────────────────────────────────────
// Simple method: each valid receiver gives one bearing estimate.
// Bearing is the receiver's known angular position on the robot.
// For small receiver separations relative to target distance,
// the target direction ≈ the direction the receiver faces.
uint8_t sonar_to_points(const SonarResult &result,
                          Point2D *points, uint8_t max_points) {
    uint8_t count = 0;
    for (uint8_t ch = 0; ch < NUM_RX && count < max_points; ch++) {
        if (!result.valid[ch]) continue;

        // Bearing: angle from robot centre to receiver, used as
        // the approximate bearing to the target.
        float bearing = atan2f(RX_Y[ch], RX_X[ch]);
        float dist    = result.distances[ch];

        // Convert polar (dist, bearing) to Cartesian
        // Offset by receiver position for accuracy
        points[count].x = RX_X[ch] + dist * cosf(bearing);
        points[count].y = RX_Y[ch] + dist * sinf(bearing);
        count++;
    }
    return count;
}

// ── sonar_tdoa ───────────────────────────────────────────────
// Time-Difference-Of-Arrival triangulation.
// With 4 receivers we get 6 receiver pairs. Each pair gives a
// hyperbola. We pick the 3 strongest echoes and solve the
// 2-receiver case (simplified, assumes target is far from robot).
bool sonar_tdoa(const SonarResult &result, Point2D &point) {
    // Find the two receivers with the smallest (closest) valid distances
    int8_t r0 = -1, r1 = -1;
    float  d0 = 1e9f, d1 = 1e9f;

    for (uint8_t ch = 0; ch < NUM_RX; ch++) {
        if (!result.valid[ch]) continue;
        if (result.distances[ch] < d0) {
            d1 = d0; r1 = r0;
            d0 = result.distances[ch]; r0 = ch;
        } else if (result.distances[ch] < d1) {
            d1 = result.distances[ch]; r1 = ch;
        }
    }

    if (r0 < 0 || r1 < 0) return false;  // Need at least 2 valid echoes

    // TDOA between the two closest receivers
    float delta_t = (result.distances[r1] - result.distances[r0]) / SPEED_OF_SOUND;

    // Baseline vector between receivers
    float bx = RX_X[r1] - RX_X[r0];
    float by = RX_Y[r1] - RX_Y[r0];
    float baseline = sqrtf(bx * bx + by * by);

    // Angle of arrival relative to the baseline
    // sin(θ) = (c × Δt) / baseline
    float sin_theta = (SPEED_OF_SOUND * delta_t) / baseline;
    sin_theta = constrain(sin_theta, -1.0f, 1.0f);
    float theta = asinf(sin_theta);

    // Baseline direction angle
    float phi = atan2f(by, bx);

    // Target bearing = baseline direction ± theta
    float bearing = phi + theta;
    float dist     = (result.distances[r0] + result.distances[r1]) / 2.0f;

    // Average receiver position as origin
    float ox = (RX_X[r0] + RX_X[r1]) / 2.0f;
    float oy = (RX_Y[r0] + RX_Y[r1]) / 2.0f;

    point.x = ox + dist * cosf(bearing);
    point.y = oy + dist * sinf(bearing);
    return true;
}

// ── sonar_serial_output ──────────────────────────────────────
void sonar_serial_output(const SonarResult &result) {
    uint32_t ts = millis();
    uint8_t  valid_mask = 0;
    for (uint8_t ch = 0; ch < NUM_RX; ch++) {
        if (result.valid[ch]) valid_mask |= (1 << ch);
    }

#if OUTPUT_CSV
    Serial.printf("%lu", ts);
    for (uint8_t ch = 0; ch < NUM_RX; ch++) {
        Serial.printf(",%.3f", result.distances[ch]);
    }
    Serial.printf(",%d\n", valid_mask);
#else
    // JSON
    Serial.printf("{\"t\":%lu,\"rx\":[", ts);
    for (uint8_t ch = 0; ch < NUM_RX; ch++) {
        Serial.printf("%.3f", result.distances[ch]);
        if (ch < NUM_RX - 1) Serial.print(',');
    }
    Serial.printf("],\"v\":%d}\n", valid_mask);
#endif
}

// ── sonar_print_points ───────────────────────────────────────
void sonar_print_points(const Point2D *points, uint8_t count) {
    for (uint8_t i = 0; i < count; i++) {
        Serial.printf("POINT,%.3f,%.3f\n", points[i].x, points[i].y);
    }
}
