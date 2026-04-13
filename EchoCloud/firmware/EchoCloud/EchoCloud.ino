// ============================================================
//  EchoCloud – Main Sketch
//  Multi-Receiver Acoustic Point-Cloud Sonar for ESP32
//  Board : ESP32 Dev Module (esp32:esp32:esp32)
//  v1.0  2026-04-13
//
//  ASCII FLOWCHART – SYSTEM OVERVIEW
//
//   ┌────────────────────────────────────────────┐
//   │                  setup()                   │
//   │  sonar_init()  →  Serial begin  →  Ready   │
//   └────────────────────┬───────────────────────┘
//                        │
//                        ▼
//   ┌────────────────────────────────────────────┐
//   │                  loop()                    │
//   │                                            │
//   │  ┌─────────────────────────────────────┐   │
//   │  │  sonar_ping_and_sample()            │   │
//   │  │  ┌─────────────┐                   │   │
//   │  │  │  TX burst   │ 500 µs @ 40 kHz  │   │
//   │  │  └──────┬──────┘                   │   │
//   │  │         │                           │   │
//   │  │  ┌──────▼──────────────────────┐   │   │
//   │  │  │ ADC sample loop (600 pts)   │   │   │
//   │  │  │  for each sample:           │   │   │
//   │  │  │   read RX0, RX1, RX2, RX3  │   │   │
//   │  │  │   (4× oversample & avg)    │   │   │
//   │  │  └──────────────────────────┘   │   │
//   │  └─────────────────────────────────┘   │
//   │                                            │
//   │  ┌─────────────────────────────────────┐   │
//   │  │  sonar_process()                    │   │
//   │  │  for each channel:                  │   │
//   │  │   estimate noise floor              │   │
//   │  │   sliding-window cross-correlation  │   │
//   │  │   find echo peak  →  compute dist   │   │
//   │  └─────────────────────────────────────┘   │
//   │                                            │
//   │  ┌─────────────────────────────────────┐   │
//   │  │  Build point cloud                  │   │
//   │  │  sonar_to_points()  (bearing method)│   │
//   │  │  sonar_tdoa()       (TDOA method)   │   │
//   │  └─────────────────────────────────────┘   │
//   │                                            │
//   │  sonar_serial_output()  — CSV / JSON        │
//   │  sonar_print_points()   — POINT,x,y        │
//   │                                            │
//   │  delay(SCAN_INTERVAL_MS)                   │
//   └────────────────────┬───────────────────────┘
//                        │ (repeat forever)
//                        └──────────────────► loop
//
// ============================================================
#include <Arduino.h>
#include "config.h"
#include "sonar.h"

#if SERVO_ENABLED
  #include <ESP32Servo.h>
  Servo scanServo;
  int   servoAngle = SERVO_MIN_DEG;
  int   servoDir   = 1;  // +1 = sweeping up, -1 = sweeping down
#endif

// Working buffers — allocated globally to avoid stack overflow
static SonarBuffer buf;
static SonarResult result;
static Point2D     points[NUM_RX + 1];  // +1 for TDOA point

// ── Forward declarations ─────────────────────────────────────
#if SERVO_ENABLED
static void servo_sweep_step(void);
#endif
static void print_header(void);

// ── setup ────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    // Wait for USB-serial to stabilise (important on ESP32 DevKit)
    delay(500);

    print_header();
    sonar_init();

#if SERVO_ENABLED
    scanServo.attach(SERVO_PIN);
    scanServo.write(SERVO_MIN_DEG);
    delay(500);  // Allow servo to reach start position
    Serial.println(F("# Servo scan enabled"));
#endif

    Serial.println(F("# READY — Starting scans"));

#if OUTPUT_CSV
    // Print CSV header
    Serial.print(F("timestamp_ms"));
    for (uint8_t ch = 0; ch < NUM_RX; ch++) {
        Serial.printf(",rx%d_m", ch);
    }
    Serial.println(F(",valid_mask"));
#endif
}

// ── loop ─────────────────────────────────────────────────────
void loop() {
    // ── Step 1: Transmit ping, capture all receiver echoes ──
    sonar_ping_and_sample(buf);

    // ── Step 2: Process raw ADC data → distances ────────────
    sonar_process(buf, result);

    // ── Step 3a: Output raw distance data (CSV / JSON) ──────
    sonar_serial_output(result);

    // ── Step 3b: Build and output 2D point cloud ────────────
    uint8_t n_bearing = sonar_to_points(result, points, NUM_RX);
    sonar_print_points(points, n_bearing);

    // TDOA: add one extra refined point if we have 2+ valid echoes
    Point2D tdoa_pt;
    if (sonar_tdoa(result, tdoa_pt)) {
        Serial.printf("TDOA,%.3f,%.3f\n", tdoa_pt.x, tdoa_pt.y);
    }

    // ── Step 4: Advance servo (if enabled) ──────────────────
#if SERVO_ENABLED
    servo_sweep_step();
#endif

    // ── Step 5: Wait before next ping ───────────────────────
    // (prevents echoes from previous ping contaminating next)
    delay(SCAN_INTERVAL_MS);
}

// ── Servo sweep helper ────────────────────────────────────────
#if SERVO_ENABLED
static void servo_sweep_step(void) {
    servoAngle += servoDir * SERVO_STEP_DEG;
    if (servoAngle >= SERVO_MAX_DEG) { servoAngle = SERVO_MAX_DEG; servoDir = -1; }
    if (servoAngle <= SERVO_MIN_DEG) { servoAngle = SERVO_MIN_DEG; servoDir =  1; }
    scanServo.write(servoAngle);
    delay(SERVO_SETTLE_MS);
    // Print current servo angle for the visualiser
    Serial.printf("SERVO,%d\n", servoAngle);
}
#endif

// ── Boot banner ───────────────────────────────────────────────
static void print_header(void) {
    Serial.println(F("#"));
    Serial.println(F("# ╔══════════════════════════════════════╗"));
    Serial.println(F("# ║  EchoCloud v1.0  — ESP32 Acoustic   ║"));
    Serial.println(F("# ║  Multi-Receiver Point-Cloud Sonar   ║"));
    Serial.println(F("# ╚══════════════════════════════════════╝"));
    Serial.printf( "# TX pin: %d   Freq: %d Hz   Burst: %d µs\n",
                   TX_PIN, TX_FREQ_HZ, TX_BURST_US);
    Serial.printf( "# Receivers: %d   MaxRange: %.0f m\n",
                   NUM_RX, 20.0f);
    Serial.printf( "# ADC res: %d-bit   Oversample: %d×\n",
                   ADC_RESOLUTION, ADC_OVERSAMPLE);
    Serial.println(F("#"));
}
