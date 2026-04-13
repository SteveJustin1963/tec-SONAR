// ============================================================
//  EchoCloud_Nano – Sonar Module Implementation
//  Arduino Nano / ATmega328P version
// ============================================================
#include "sonar_nano.h"
#include <math.h>
#include <avr/io.h>

// Receiver pin table
static const uint8_t RX_PINS_N[NUM_RX_NANO] = {
    RX_PIN_0, RX_PIN_1, RX_PIN_2, RX_PIN_3
};

// Receiver physical positions on robot (metres)
static const float RX_X_N[NUM_RX_NANO] = RX_POS_X_NANO;
static const float RX_Y_N[NUM_RX_NANO] = RX_POS_Y_NANO;

// ── sonar_nano_free_ram ───────────────────────────────────────
// Standard AVR trick: SP (stack pointer) grows down from 0x08FF;
// heap grows up from __heap_start. The gap between them is free.
extern unsigned int __heap_start;
extern void *__brkval;

int sonar_nano_free_ram(void) {
    int free_memory;
    if (__brkval == 0) {
        free_memory = ((int)&free_memory) - ((int)&__heap_start);
    } else {
        free_memory = ((int)&free_memory) - ((int)__brkval);
    }
    return free_memory;
}

// ── sonar_nano_init ───────────────────────────────────────────
void sonar_nano_init(void) {
    // ── Timer1: Fast PWM, ICR1 as TOP, OC1A (pin 9) as output ──
    //  Registers:
    //   TCCR1A: COM1A1=1 (clear OC1A on compare match),
    //           WGM11=1  (Fast PWM, ICR1 as TOP — part 1)
    //   TCCR1B: WGM13=1, WGM12=1 (Fast PWM — part 2),
    //           CS10=0   (clock stopped until burst start)
    //   ICR1  : TOP = 399  →  f = 16MHz / 400 = 40,000 Hz exactly
    //   OCR1A : 200        →  50 % duty cycle
    //
    //  We leave the clock stopped (CS1x = 000) until sonar fires.
    //  pinMode is set but output stays low because OC1A only
    //  drives when TCCR1B has a clock source.
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    ICR1   = TIMER1_ICR1;
    OCR1A  = TIMER1_OCR1A;
    // WGM1[3:0] = 1110 (Fast PWM, ICR1 = TOP)
    // COM1A[1:0] = 10  (Clear OC1A on compare match, non-inverting)
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12);
    // Note: CS10 not yet set → counter stopped → no PWM output yet

    // Make pin 9 (OC1A) an output AFTER configuring timer
    pinMode(TX_PIN_NANO, OUTPUT);
    digitalWrite(TX_PIN_NANO, LOW);

    // ADC pins — Nano ADC is inherently input-only; default reference = AVcc = 5V
    // No extra init needed; analogRead() works out of the box.
    // The envelope detector output is ≤ 5V, matching Nano's 5V ADC reference.

    pinMode(LED_PIN_NANO, OUTPUT);
    digitalWrite(LED_PIN_NANO, LOW);

    Serial.println(F("# EchoCloud_Nano v1.0"));
    Serial.print(F("# TX=pin9 (Timer1 OC1A) Burst="));
    Serial.print(TX_BURST_US);
    Serial.print(F(" us Channels="));
    Serial.println(NUM_RX_NANO);
#if DEBUG_RAM_NANO
    Serial.print(F("# Free RAM: "));
    Serial.print(sonar_nano_free_ram());
    Serial.println(F(" bytes"));
#endif
}

// ── sonar_nano_ping_and_sample ────────────────────────────────
void sonar_nano_ping_and_sample(SonarBufferNano &buf) {
    // ── 1. Transmit: start Timer1 clock → 40 kHz on pin 9 ───
    digitalWrite(LED_PIN_NANO, HIGH);
    TCNT1  = 0;                          // Reset counter
    TCCR1B |= (1 << CS10);              // Start clock (no prescaler)
    delayMicroseconds(TX_BURST_US);      // Burst duration (500 µs = 20 cycles)
    TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10)); // Stop clock
    digitalWrite(TX_PIN_NANO, LOW);      // Drive pin low after burst
    digitalWrite(LED_PIN_NANO, LOW);

    // ── 2. Sample all receivers as fast as possible ──────────
    //  Store only top 8 bits of 10-bit ADC (>> 2) to save RAM.
    uint32_t t_start = micros();
    for (uint16_t s = 0; s < MAX_SAMPLES_NANO; s++) {
        for (uint8_t ch = 0; ch < NUM_RX_NANO; ch++) {
            buf.data[ch][s] = (uint8_t)(analogRead(RX_PINS_N[ch]) >> 2);
        }
    }
    uint32_t elapsed = micros() - t_start;

    buf.num_samples      = MAX_SAMPLES_NANO;
    buf.sample_period_us = (uint16_t)(elapsed / MAX_SAMPLES_NANO);

#if DEBUG_TIMING_NANO
    Serial.print(F("# sample_period="));
    Serial.print(buf.sample_period_us);
    Serial.print(F(" us  rate="));
    Serial.print(1000000UL / buf.sample_period_us);
    Serial.println(F(" sps"));
#endif
}

// ── sonar_nano_noise_floor ────────────────────────────────────
uint8_t sonar_nano_noise_floor(const uint8_t *channel, uint8_t n) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < n; i++) sum += channel[i];
    return (uint8_t)(sum / n);
}

// ── sonar_nano_find_echo ──────────────────────────────────────
// Sliding-window cross-correlation with rectangular pulse.
// Uses 16-bit accumulator — no overflow: 8-bit × up to 20 samples
// → max 255 × 20 = 5100, fits in uint16_t.
int16_t sonar_nano_find_echo(const uint8_t *channel,
                              uint16_t num_samples,
                              uint8_t  pulse_samples,
                              uint8_t  threshold) {
    if (num_samples < (uint16_t)(DEAD_ZONE_NANO + pulse_samples)) {
        return -1;
    }

    uint16_t search_start = DEAD_ZONE_NANO;
    uint16_t search_end   = num_samples - pulse_samples;

    // Build initial window
    uint16_t win_sum = 0;
    for (uint8_t k = 0; k < pulse_samples; k++) {
        win_sum += channel[search_start + k];
    }

    uint16_t best_sum = 0;
    uint16_t best_idx = search_start;

    for (uint16_t lag = search_start; lag < search_end; lag++) {
        if (win_sum > best_sum) {
            best_sum = win_sum;
            best_idx = lag;
        }
        // Slide window
        win_sum -= channel[lag];
        win_sum += channel[lag + pulse_samples];
    }

    // Check threshold: sum must exceed threshold × pulse_samples
    uint16_t min_sum = (uint16_t)threshold * pulse_samples;
    if (best_sum < min_sum) return -1;

    return (int16_t)(best_idx + pulse_samples / 2);
}

// ── sonar_nano_process ────────────────────────────────────────
void sonar_nano_process(const SonarBufferNano &buf,
                         SonarResultNano &result) {
    result.sample_rate_hz = (buf.sample_period_us > 0)
        ? 1000000.0f / (float)buf.sample_period_us
        : 2232.0f;  // Fallback: typical Nano rate with 4 channels

    // Pulse width in samples
    uint8_t pulse_samples = (uint8_t)(
        (float)TX_BURST_US / (float)buf.sample_period_us
    );
    if (pulse_samples < 1) pulse_samples = 1;
    if (pulse_samples > 20) pulse_samples = 20;  // Sanity cap

    for (uint8_t ch = 0; ch < NUM_RX_NANO; ch++) {
        uint8_t noise     = sonar_nano_noise_floor(buf.data[ch], DEAD_ZONE_NANO);
        uint8_t threshold = (uint8_t)min((uint16_t)255,
                                         (uint16_t)noise * THRESHOLD_MULT_NANO);

        int16_t echo_idx = sonar_nano_find_echo(
            buf.data[ch], buf.num_samples, pulse_samples, threshold
        );

        if (echo_idx < 0) {
            result.distances[ch] = -1.0f;
            result.valid[ch]     = false;
        } else {
            float round_trip_s    = (float)echo_idx / result.sample_rate_hz;
            result.distances[ch]  = (round_trip_s * SPEED_OF_SOUND_F) / 2.0f;
            result.valid[ch]      = (result.distances[ch] > 0.05f &&
                                     result.distances[ch] < 15.0f);
        }
    }
}

// ── sonar_nano_to_points ──────────────────────────────────────
uint8_t sonar_nano_to_points(const SonarResultNano &result,
                               Point2DNano *points,
                               uint8_t max_points) {
    uint8_t count = 0;
    for (uint8_t ch = 0; ch < NUM_RX_NANO && count < max_points; ch++) {
        if (!result.valid[ch]) continue;
        float bearing  = atan2(RX_Y_N[ch], RX_X_N[ch]);
        float dist     = result.distances[ch];
        points[count].x = RX_X_N[ch] + dist * cos(bearing);
        points[count].y = RX_Y_N[ch] + dist * sin(bearing);
        count++;
    }
    return count;
}

// ── sonar_nano_serial_output ──────────────────────────────────
void sonar_nano_serial_output(const SonarResultNano &result) {
    uint8_t valid_mask = 0;
    for (uint8_t ch = 0; ch < NUM_RX_NANO; ch++) {
        if (result.valid[ch]) valid_mask |= (1 << ch);
    }

#if OUTPUT_CSV_NANO
    Serial.print(millis());
    for (uint8_t ch = 0; ch < NUM_RX_NANO; ch++) {
        Serial.print(',');
        // dtostrf: AVR-safe float→string  (avoids printf %f on small AVR)
        char buf[8];
        dtostrf(result.distances[ch], 6, 3, buf);
        Serial.print(buf);
    }
    Serial.print(',');
    Serial.println(valid_mask);
#else
    // JSON — uses less RAM than printf on AVR
    Serial.print(F("{\"t\":"));
    Serial.print(millis());
    Serial.print(F(",\"rx\":["));
    for (uint8_t ch = 0; ch < NUM_RX_NANO; ch++) {
        char buf[8];
        dtostrf(result.distances[ch], 6, 3, buf);
        Serial.print(buf);
        if (ch < NUM_RX_NANO - 1) Serial.print(',');
    }
    Serial.print(F("],\"v\":"));
    Serial.print(valid_mask);
    Serial.println('}');
#endif
}

// ── sonar_nano_print_points ───────────────────────────────────
void sonar_nano_print_points(const Point2DNano *points, uint8_t count) {
    char xbuf[8], ybuf[8];
    for (uint8_t i = 0; i < count; i++) {
        dtostrf(points[i].x, 6, 3, xbuf);
        dtostrf(points[i].y, 6, 3, ybuf);
        Serial.print(F("POINT,"));
        Serial.print(xbuf);
        Serial.print(',');
        Serial.println(ybuf);
    }
}
