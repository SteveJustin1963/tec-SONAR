# EchoCloud
## Multi-Receiver Acoustic Point-Cloud Sonar for Robot Navigation

> **One ultrasonic transmitter. Four receivers. A live 2-D point cloud on your screen.**
> An ESP32-powered acoustic "LIDAR" built from DSP first-principles.

---

## Table of Contents

1. [Why EchoCloud?](#1-why-echocloud)
2. [How It Works — Theory](#2-how-it-works--theory)
3. [Hardware — Bill of Materials + Prices](#3-hardware--bill-of-materials--prices)
4. [Circuit Design](#4-circuit-design)
5. [Wiring Guide](#5-wiring-guide)
6. [Software Architecture & ASCII Flowcharts](#6-software-architecture--ascii-flowcharts)
7. [Installation & Setup](#7-installation--setup)
8. [Running the Project](#8-running-the-project)
9. [Output Format](#9-output-format)
10. [Troubleshooting](#10-troubleshooting)
11. [Performance & Limitations](#11-performance--limitations)
12. [Arduino Nano Version](#12-arduino-nano-version)
13. [Future Improvements](#13-future-improvements)
14. [Theory References](#14-theory-references)

---

## 1. Why EchoCloud?

### The Problem with Cheap Robots

Most DIY robots have one sensor — a single HC-SR04 ultrasonic sensor pointing forward. It answers one question: *"Is there anything in front of me?"*. It gives no direction information, no side awareness, and no map. It reacts, it doesn't plan.

Real mobile robots (Roomba, ROS bots, warehouse AGVs) use **2D LIDAR** — a spinning laser rangefinder that builds a complete map slice. But even the cheapest laser LIDARs cost AU$80–200 and are black-box modules that don't teach you anything.

### The Idea

The core DSP insight (from *Digital Signal Processing using MATLAB*, Ingle & Proakis, Chapter 2) is this:

> **Cross-correlation between a known transmitted signal and a received signal gives the time delay of the echo precisely.**

A bat does exactly this. So can we.

EchoCloud uses:
- **One 40 kHz ultrasonic transmitter** — fires a short chirp burst in all directions.
- **Four 40 kHz receivers** placed around the robot — each hears the echo arriving at a slightly different time.
- **ESP32 ADC** — samples all four receiver channels simultaneously.
- **Cross-correlation** — finds the precise delay for each channel → distance.
- **Geometry** — converts distances + receiver positions → (x, y) point cloud.
- **Python matplotlib** — draws the live map.

**Result:** A 360° acoustic point cloud, range 0.5–15 m, ~5 cm resolution, updating 5× per second — for under AU$35.

### Why ESP32?

| Feature | Arduino Uno | ESP32 |
|---------|-------------|-------|
| Clock speed | 16 MHz | 240 MHz |
| RAM | 2 KB | 520 KB |
| ADC channels | 6 | 18 (12-bit) |
| Serial speed | 115200 baud | 115200+ |
| WiFi (future) | No | Yes (built-in) |
| Cost | ~AU$5 | ~AU$5 |

The ESP32 can hold 4 × 600 = 2400 ADC samples in RAM (4.8 KB), run cross-correlation at 240 MHz, and stream results over WiFi in future versions — all for the same price as an Arduino.

---

## 2. How It Works — Theory

### 2.1 Acoustic Ranging — Time of Flight

Sound travels at **343 m/s** at 20 °C. If we fire a pulse and time its echo:

```
distance = (time_of_flight × speed_of_sound) / 2
```

The `/2` is because the sound travels *there and back*. At 343 m/s:
- 1 metre ≈ 5.83 ms round trip
- 20 metre ≈ 116.6 ms round trip
- Time resolution of 200 µs → ~3.4 cm range resolution

### 2.2 The Transmitted Burst

We fire **20 cycles of 40 kHz** from a piezoelectric transducer:
```
Burst duration = 20 cycles × (1 / 40,000 Hz) = 500 µs
```

40 kHz is used because:
- Above human hearing (inaudible)
- Cheap piezo transducers available
- Good directivity at 16 mm element size
- Reasonable range in air (vs. MHz ultrasound which attenuates quickly)

### 2.3 Cross-Correlation — The DSP Heart

Cross-correlation measures *how similar* two signals are at different time offsets:

```
(x ⋆ y)[τ] = Σ x[t] · y[t + τ]
             t
```

When the transmitted signal `x` appears in the received signal `y` at delay `τ₀`, the cross-correlation peaks at `τ₀`. This gives the echo time precisely, even in the presence of noise.

**In EchoCloud**, the transmitted burst creates a roughly rectangular envelope (a pulse `TX_BURST_US` wide). We cross-correlate this template with the received ADC envelope. The peak tells us the sample index of the echo.

The implementation uses a **sliding window sum** (computationally equivalent to cross-correlation with a rectangular pulse, but O(N) instead of O(N²)):

```
ASCII DIAGRAM — Sliding Window Cross-Correlation

  Received envelope (600 samples):
  _____|¯¯¯¯|_______________|¯¯¯|________
       ^                    ^
       direct blast         echo peak
       (dead zone)          ← we find this

  Window sum as it slides right:
       ████████            ▓▓▓▓▓▓▓▓
  ↑ low baseline      ↑ rises sharply at echo
```

The peak of the sliding window sum = the cross-correlation maximum = echo position.

### 2.4 Envelope Detection

The 40 kHz signal is too fast to directly sample (Nyquist: 80 ksps minimum). Instead, we detect its **envelope** — a slow-moving amplitude signal — with an analogue circuit:

```
  40 kHz carrier         Envelope
  ~~~~~~~~~~~    →    ___/¯¯¯¯¯\___
  received             ^ rises when burst arrives
  from transducer
```

The envelope detector uses:
- A diode half-wave rectifier (1N4148)
- An RC low-pass filter (R = 1 kΩ, C = 10 nF → τ = 10 µs)

This lets the ESP32 ADC sample at just ~5 ksps instead of 80+ ksps.

### 2.5 Time-Difference of Arrival (TDOA)

When a receiver closer to the target hears the echo before a farther receiver, the time *difference* reveals the angle:

```
                 TARGET
                   *
                  /|\
                 / | \
                /  |  \
  RX0 ●────── d0   d1 ──────● RX1
              |←── Δd ──→|

  Δt = Δd / c              (speed of sound c = 343 m/s)
  sin(θ) = c·Δt / baseline
  θ = arcsin(c·Δt / d_baseline)
```

With receiver baseline = 0.10 m (10 cm), and timing resolution of 200 µs:
- Angle resolution ≈ arcsin(343 × 200e-6 / 0.10) ≈ arcsin(0.686) ≈ 43°

TDOA angle resolution improves significantly with faster sampling or wider baseline.

### 2.6 Point Cloud Construction

For each valid receiver echo we produce one (x, y) point:

```python
bearing = atan2(ry[ch], rx[ch])          # receiver's mounting angle
x = rx_pos_x[ch] + distance * cos(bearing)
y = rx_pos_y[ch] + distance * sin(bearing)
```

Plotting these 4+ points per ping over many pings at different servo angles produces a sparse but real 2D map of the environment.

### 2.7 Comparison to Laser LIDAR

| Property | EchoCloud (acoustic) | RPLidar A1 (laser) |
|----------|---------------------|---------------------|
| Range | 0.5–15 m | 0.15–12 m |
| Angular res. | ~45° / ping | 1° |
| Update rate | 5 pings/s | 5.5 Hz (full 360°) |
| Range res. | ~3.4 cm | ~1 cm |
| Affected by | Soft surfaces, porous materials | Mirrors, black surfaces |
| Cost (AU) | ~$35 | ~$130 |
| Teaches DSP? | **YES** | No |

EchoCloud is not a LIDAR replacement — it's a cheap, educational sensor that covers the same job at robot-navigation accuracy.

---

## 3. Hardware — Bill of Materials + Prices

All prices are approximate AliExpress/eBay prices (April 2026) in AUD.
Local electronics stores (Jaycar, Element14) will cost 2–3× more but arrive faster.

### Core Components

| # | Component | Spec / Notes | Qty | Unit (AU$) | Total |
|---|-----------|-------------|-----|------------|-------|
| 1 | ESP32 DevKit V1 | WROOM-32, 38-pin | 1 | $5.50 | $5.50 |
| 2 | 40 kHz Ultrasonic TX | Murata MA40S4S *or* generic open-frame 40 kHz | 1 | $3.50 | $3.50 |
| 3 | 40 kHz Ultrasonic RX | Murata MA40S4R *or* matching pair | 4 | $2.50 | $10.00 |
| 4 | LM358N dual op-amp | DIP-8, 5V single supply | 2 | $0.50 | $1.00 |
| 5 | 1N4148 signal diode | Fast switching, envelope detector | 4 | $0.10 | $0.40 |
| 6 | 2N2222 NPN transistor | Tx buffer, TO-92 | 1 | $0.15 | $0.15 |
| 7 | Resistors | 100Ω, 1kΩ, 10kΩ, 39kΩ, 470kΩ (5× each) | 1 bag | $1.00 | $1.00 |
| 8 | Capacitors | 10nF, 100nF, 10µF (5× each) | 1 bag | $1.00 | $1.00 |
| 9 | Breadboard | 830-point | 1 | $2.50 | $2.50 |
| 10 | Jumper wires | M-M, 20 cm, 40-piece | 1 pack | $2.00 | $2.00 |
| 11 | USB-A to micro-USB | Programming cable | 1 | $1.50 | $1.50 |

**Core total: ~AU$28.55**

### Optional (Servo Scan / Full Robot)

| # | Component | Spec | Qty | Unit | Total |
|---|-----------|------|-----|------|-------|
| 12 | SG90 micro servo | 180° pan | 1 | $3.00 | $3.00 |
| 13 | SG90 micro servo | 180° tilt (3D scan) | 1 | $3.00 | $3.00 |
| 14 | 2WD robot chassis | Acrylic, includes motors+wheels | 1 | $14.00 | $14.00 |
| 15 | L298N motor driver | Dual H-bridge | 1 | $3.00 | $3.00 |
| 16 | 18650 LiPo battery | 3.7V 2600mAh + holder | 2 | $4.00 | $8.00 |
| 17 | TP4056 charger module | USB-C LiPo charger | 1 | $1.50 | $1.50 |
| 18 | MT3608 boost converter | 3.7V → 5V @ 2A | 1 | $1.50 | $1.50 |
| 19 | DHT22 sensor | Temperature correction for c | 1 | $2.50 | $2.50 |
| 20 | 0.96" OLED display | I²C SSD1306, range readout | 1 | $3.00 | $3.00 |

**Full robot total: ~AU$65.05**

### Budget Alternative (Dismantle HC-SR04 modules)

HC-SR04 modules include one TX and one RX transducer each. You can:
1. Buy 5× HC-SR04 modules (~$1 each = $5 total)
2. Desolder or directly use the TX from module 1
3. Use the RX transducers from modules 2–5
4. Use the HC-SR04 trigger/echo circuit as a pre-built amplifier

**Budget alternative: ~AU$10 for transducers**

---

## 4. Circuit Design

### 4.1 Transmitter Circuit

The ESP32 GPIO outputs 3.3 V at up to 40 mA. A 40 kHz piezo transducer needs 50–100 mA peak for adequate sound pressure. We use a 2N2222 NPN transistor as a current buffer:

```
                          +5V (USB VBUS)
                           │
                          [100Ω]   ← current limiter
                           │
                    ┌──────┤(+) MA40S4S Transducer (−)──┐
                    │      │                              │
                    │   C (collector)                     │
ESP32 GPIO26 ──[100Ω]── B (base) 2N2222                  │
                         E (emitter) ──────────────────GND┘
```

**Operation:**
- GPIO26 outputs 40 kHz PWM (50% duty, via ESP32 LEDC peripheral)
- When HIGH: transistor saturates, transducer drives current from +5V → GND
- When LOW: transistor cuts off, transducer is silent
- Peak current: (5V − 0.2V sat) / 100Ω ≈ 48 mA ✓

**Why 5V for the transmitter?** Higher voltage = higher sound pressure level (SPL). More SPL = longer range. The ESP32 itself runs at 3.3V; we just use the USB 5V rail for the TX only.

### 4.2 Receiver Amplifier Circuit (per channel, repeat × 4)

Raw piezo receiver output is typically 1–50 mV. The ESP32 ADC needs at least 100 mV for reliable readings. We use one half of an LM358 op-amp for 40× amplification:

```
                        +5V
                         │
                        [1MΩ]  ← bias to mid-supply
                         │
  MA40S4R (+) ──[0.1µF]──┤──── (+) IN of LM358 non-inv
  MA40S4R (−) ────────── GND        │
                                  LM358
                               (−) IN ──┬── Output
                                        │
                                       [39kΩ]
                                        │
                                       [1kΩ] ── GND

  LM358 VCC = +5V,  GND = GND
  Gain = 1 + 39k/1k = 40  (32 dB)
```

**Why LM358?** Cheap (AU$0.50), works on single 5V supply, output swings near 0V and to ~3.5V. Available everywhere.

**Why 0.1 µF coupling cap?** Blocks DC bias from the transducer. Allows 40 kHz signal through (Xc = 1/(2π×40k×0.1µ) ≈ 40 Ω — negligible).

### 4.3 Envelope Detector Circuit (per channel)

The 40 kHz amplified signal is converted to a slow-moving envelope for the ESP32 ADC:

```
  LM358 Output
       │
      [1kΩ]           ESP32 ADC pin
       │                    │
       ├──(1N4148 →)──┬────[10kΩ]──┤
       │              │            │
       │            [10nF]       [10kΩ]
       │              │            │
      GND            GND          GND

  Complete envelope detector + voltage divider:
  - 1N4148: half-wave rectification (peak detection)
  - 10nF: smoothing cap  (τ_attack ≈ 10µs)
  - First 10kΩ + second 10kΩ: voltage divider (5V → 2.5V max for ADC)
  - Final signal at ADC: 0 – ~2.5V DC envelope
```

**RC time constant:** τ = R × C = 1kΩ × 10nF = 10 µs
- Fast enough to follow 40 kHz envelope rise (rise time of burst = 25 µs)
- Slow enough to bridge individual cycles (period = 25 µs)

**Voltage divider:** Brings the 5V op-amp output to ≤ 2.5V for the ESP32 ADC (rated 3.3V max input).

### 4.4 Full Signal Chain (ASCII)

```
  ┌─ TX CHAIN ──────────────────────────────────────────────────┐
  │                                                              │
  │  ESP32         2N2222       MA40S4S                          │
  │  GPIO26 ─[100Ω]─[B]─[C]─[100Ω]─[TX(+)]─[TX(−)]─GND       │
  │                      [E]─GND                                 │
  │                   +5V USB                                    │
  └──────────────────────────────────────────────────────────────┘

  ┌─ RX CHAIN (×4) ─────────────────────────────────────────────┐
  │                                                              │
  │  MA40S4R    Coupling    Amp         Rectifier    Divider     │
  │  [RX(+)]─[0.1µF]─[LM358(40×)]─[1N4148]─[10k/10k]─ADC pin │
  │  [RX(−)]─GND      │                  │                      │
  │                  [1MΩ]─5V           [10nF]─GND              │
  │                  [39k/1k feedback]                           │
  └──────────────────────────────────────────────────────────────┘

  ┌─ ESP32 ────────────────────────────────────────────────────┐
  │  GPIO26 → TX burst (LEDC 40kHz PWM)                        │
  │  GPIO36 → RX0 ADC (FL)   GPIO39 → RX1 ADC (FR)            │
  │  GPIO34 → RX2 ADC (RL)   GPIO35 → RX3 ADC (RR)            │
  │  GPIO18 → Servo signal (optional)                           │
  │  GPIO2  → LED (status)                                      │
  └────────────────────────────────────────────────────────────┘
```

---

## 5. Wiring Guide

### ESP32 DevKit V1 Pinout (WROOM-32, 38-pin)

```
                    ┌─────────────────┐
               EN ──┤1             38 ├── GPIO23 (MOSI)
            GPIO36 ──┤2  RX0-ADC   37 ├── GPIO22 (SCL)
            GPIO39 ──┤3  RX1-ADC   36 ├── TX
            GPIO34 ──┤4  RX2-ADC   35 ├── RX
            GPIO35 ──┤5  RX3-ADC   34 ├── GPIO21 (SDA)
            GPIO32 ──┤6            33 ├── GND
            GPIO33 ──┤7            32 ├── 5V (USB Vin)
              GND ──┤8            31 ├── GPIO19 (MISO)
              5V  ──┤9            30 ├── GPIO18 (SERVO ←)
            GPIO26 ──┤10 TX-PIN   29 ├── GPIO5
            GPIO27 ──┤11          28 ├── GPIO17
            GPIO14 ──┤12          27 ├── GPIO16
            GPIO12 ──┤13          26 ├── GPIO4
             GPIO13 ──┤14          25 ├── GPIO0
             GND  ──┤15          24 ├── GPIO2 (LED ←)
             3.3V ──┤16          23 ├── GPIO15
            GPIO1  ──┤17 (USB TX) 22 ├── GPIO8
            GPIO3  ──┤18 (USB RX) 21 ├── GPIO7
            GPIO2  ──┤19          20 ├── GPIO6
                    └─────────────────┘
```

### Step-by-Step Wiring

```
POWER:
  USB-C/micro cable → ESP32 USB port   (powers ESP32 @ 3.3V)
  ESP32 5V pin → LM358 VCC (+)         (powers op-amps @ 5V)
  ESP32 5V pin → 2N2222 collector rail  (TX power)
  ESP32 GND   → common ground (all components)

TRANSMITTER:
  ESP32 GPIO26 → 100Ω resistor → 2N2222 base (pin 1)
  2N2222 emitter (pin 3) → GND
  2N2222 collector (pin 2) → 100Ω → MA40S4S (+) pin
  MA40S4S (-) pin → GND

RECEIVER 0 (FRONT-LEFT, GPIO36):
  MA40S4R(0) (+) → 0.1µF cap → LM358(A) pin3 (+IN)
  MA40S4R(0) (-) → GND
  1MΩ resistor between LM358(A) pin3 and 5V   (bias)
  LM358(A) pin2 (-IN) → junction of 39kΩ–1kΩ voltage divider (to GND)
  LM358(A) pin1 (OUT) → top of 39kΩ
  LM358(A) pin1 (OUT) → 1kΩ → 1N4148 anode
  1N4148 cathode → node A
  Node A → 10nF cap → GND
  Node A → 10kΩ → node B
  Node B → 10kΩ → GND
  Node B → ESP32 GPIO36

  (Repeat identically for RX1→GPIO39, RX2→GPIO34, RX3→GPIO35)

SERVO (optional):
  ESP32 GPIO18 → SG90 signal wire (orange/yellow)
  5V → SG90 VCC (red)
  GND → SG90 GND (brown/black)
```

### Physical Receiver Placement on Robot

```
  Robot top view:

       FRONT
        ↑
   ┌────┴────┐
   │  RX0   RX1│  ← 10 cm apart (left/right)
   │  FL    FR │
   │           │
   │  Robot    │
   │  Body     │
   │           │
   │  RL    RR │
   │  RX2   RX3│  ← 10 cm apart
   └───────────┘

  RX0 (FRONT-LEFT)  → GPIO36, position (+5cm, +5cm)
  RX1 (FRONT-RIGHT) → GPIO39, position (+5cm, -5cm)
  RX2 (REAR-LEFT)   → GPIO34, position (-5cm, +5cm)
  RX3 (REAR-RIGHT)  → GPIO35, position (-5cm, -5cm)

  Emitter (TX): mount centrally at top — omnidirectional in horizontal plane.
```

---

## 6. Software Architecture & ASCII Flowcharts

### 6.1 Project File Structure

```
EchoCloud/
├── README.md                      ← This file
├── platformio.ini                 ← PlatformIO build config
├── firmware/
│   └── EchoCloud/
│       ├── EchoCloud.ino          ← Main Arduino sketch (setup + loop)
│       ├── config.h               ← All pin/parameter constants
│       ├── sonar.h                ← Sonar module API (structs + prototypes)
│       └── sonar.cpp              ← Sonar module implementation
├── python/
│   ├── requirements.txt           ← pyserial, matplotlib, numpy
│   └── echocloud_viz.py           ← Real-time point cloud visualiser
└── hardware/
    └── wiring.md                  ← Expanded wiring reference
```

### 6.2 Firmware Main Loop

```
setup()
  │
  ├─ Serial.begin(115200)
  ├─ sonar_init()
  │     ├─ ledcAttach(TX_PIN, 40000, 8-bit)
  │     ├─ ledcWrite(TX_PIN, 0)          — silent until ping
  │     ├─ analogReadResolution(12)
  │     ├─ analogSetAttenuation(ADC_11db) — 0..3.3V range
  │     └─ pinMode(RX_PINS[0..3], INPUT)
  └─ Print boot banner

loop() ─────────────────────────────────────────────────
  │
  ▼
  ┌──────────────────────────────────────────────────────┐
  │  sonar_ping_and_sample(buf)                          │
  │                                                      │
  │  ledcWrite(TX_PIN, 128)  ← start 40kHz burst        │
  │  delayMicroseconds(500)  ← 20 cycles @ 40kHz        │
  │  ledcWrite(TX_PIN, 0)    ← stop burst                │
  │                                                      │
  │  for s = 0 to 599:                                   │
  │    for ch = 0 to 3:                                  │
  │      acc = 0                                         │
  │      for ov = 0 to 3:                                │
  │        acc += analogRead(RX_PINS[ch])                │
  │      buf.data[ch][s] = acc / 4                       │
  │                                                      │
  │  record sample_period_us = elapsed / MAX_SAMPLES     │
  └──────────────────────────────────────────────────────┘
  │
  ▼
  ┌──────────────────────────────────────────────────────┐
  │  sonar_process(buf, result)                          │
  │                                                      │
  │  for each channel ch:                                │
  │    noise = mean(buf.data[ch][0..19])   ← dead zone  │
  │    threshold = noise × 3               ← 3× floor   │
  │    pulse_len = TX_BURST_US / sample_period_us        │
  │                                                      │
  │    ← sonar_find_echo(channel, ...) ──────────────── │
  │      │                                               │
  │      │  Initialize sliding window sum (len=pulse_len)│
  │      │  Slide from dead_zone to end:                 │
  │      │    track maximum window sum & its position    │
  │      │  if max_sum > threshold × pulse_len:          │
  │      │    return centre_of_best_window               │
  │      │  else: return -1 (no echo)                    │
  │      └────────────────────────────────────────────── │
  │                                                      │
  │    if echo_sample >= 0:                              │
  │      round_trip = echo_sample / sample_rate_hz       │
  │      distance = (round_trip × 343) / 2              │
  │      valid[ch] = (0.05m < distance < 20m)           │
  │    else:                                             │
  │      distance = -1.0, valid = false                  │
  └──────────────────────────────────────────────────────┘
  │
  ▼
  ┌──────────────────────────────────────────────────────┐
  │  sonar_serial_output(result)                         │
  │  → "timestamp,d0,d1,d2,d3,valid_mask\n"             │
  │                                                      │
  │  sonar_to_points(result, points)                     │
  │  → bearing = atan2(ry[ch], rx[ch])                   │
  │    x = rx_pos_x + dist × cos(bearing)               │
  │    y = rx_pos_y + dist × sin(bearing)               │
  │  sonar_print_points() → "POINT,x,y\n"               │
  │                                                      │
  │  sonar_tdoa(result, tdoa_pt)                         │
  │  → find two closest receivers                        │
  │    Δt = (d1 - d0) / 343                              │
  │    θ = arcsin(343 × Δt / baseline)                   │
  │    bearing = atan2(baseline_y, baseline_x) + θ       │
  │  → "TDOA,x,y\n"                                     │
  └──────────────────────────────────────────────────────┘
  │
  ▼
  delay(200ms)   ← prevent ping overlap
  │
  └──────────────────────────────────────────► (repeat)
```

### 6.3 Python Visualiser

```
echocloud_viz.py
  │
  ├─ parse_args()
  │    --port  /dev/ttyUSB0
  │    --baud  115200
  │    --log   output.csv
  │    --demo  (no hardware)
  │
  ├─ if --demo:  start demo_generator() thread
  │     └─ simulates room walls using ray-wall intersection
  │        adds gaussian noise, fills point_buffer
  │
  ├─ else:  open serial port → start serial_reader() thread
  │           reads lines → classifies → fills point_buffer
  │
  ├─ setup_plot()    ← matplotlib dark theme, robot marker
  │
  └─ animation.FuncAnimation → update_plot() @ 10 Hz
       │
       ├─ filter point_buffer by age (FADE_SECONDS = 5)
       ├─ split into bearing_pts (cyan) and tdoa_pts (red)
       ├─ scatter.set_offsets(np.column_stack([x, y]))
       ├─ update distance text panel
       └─ draw servo angle indicator
```

### 6.4 DSP Pipeline (per ping)

```
  Physical World        ESP32 ADC           DSP               Output
  ─────────────         ─────────           ───               ──────

  TX burst ──────────►  GPIO26 PWM
  (40kHz, 500µs)        LEDC @ 40kHz

  Acoustic              Envelope           Sliding window
  propagation           detector           cross-correlation
  in air                circuit            ┌───────────────┐
      │                     │              │ Window slides  │
      ▼                     ▼              │ right →       │     (x, y) point
  Echo at RX0 ──► ADC36 ──► uint16[] ──►  │ track max sum  │ ──► POINT,1.2,0.3
  Echo at RX1 ──► ADC39 ──► uint16[] ──►  │               │ ──► POINT,0.8,-0.1
  Echo at RX2 ──► ADC34 ──► uint16[] ──►  │ find echo idx  │ ──► POINT,...
  Echo at RX3 ──► ADC35 ──► uint16[] ──►  └───────────────┘ ──► POINT,...
                            (600 samples           │
                             per channel)          ▼
                                           idx / sample_rate
                                           = round-trip time
                                           × 343 / 2
                                           = distance (m)
```

---

## 7. Installation & Setup

### 7.1 Prerequisites

- Computer running Linux, macOS, or Windows
- Python 3.8+ with pip
- ESP32 USB driver (CP210x or CH340 — usually auto-installed on Linux/Mac)

### 7.2 Install Arduino-CLI and ESP32 Core

```bash
# Install arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
  | BINDIR=~/.local/bin sh

# Add to PATH (bash)
echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc && source ~/.bashrc

# Add ESP32 board manager URL
arduino-cli config init
arduino-cli config set board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Install ESP32 Arduino core (~200 MB download)
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

### 7.3 Install Python Dependencies

```bash
pip3 install pyserial matplotlib numpy
# or
pip3 install -r python/requirements.txt
```

### 7.4 Compile the Firmware

```bash
cd EchoCloud
arduino-cli compile \
  --fqbn esp32:esp32:esp32 \
  firmware/EchoCloud

# Expected output:
# Sketch uses 309647 bytes (23%) of program storage space.
# Global variables use 27232 bytes (8%) of dynamic memory.
```

### 7.5 Upload to ESP32

```bash
# Find your ESP32 port
arduino-cli board list

# Upload (replace /dev/ttyUSB0 with your port)
arduino-cli upload \
  --fqbn esp32:esp32:esp32 \
  --port /dev/ttyUSB0 \
  firmware/EchoCloud
```

---

## 8. Running the Project

### 8.1 Monitor Raw Serial Output

```bash
arduino-cli monitor --port /dev/ttyUSB0 --config baudrate=115200
# or
screen /dev/ttyUSB0 115200
# or
minicom -D /dev/ttyUSB0 -b 115200
```

You should see:
```
#
# ╔══════════════════════════════════════╗
# ║  EchoCloud v1.0  — ESP32 Acoustic   ║
# ╚══════════════════════════════════════╝
# READY — Starting scans
timestamp_ms,rx0_m,rx1_m,rx2_m,rx3_m,valid_mask
543,1.234,1.189,1.201,1.245,15
743,1.231,1.190,1.205,1.240,15
943,-1.000,-1.000,1.203,1.244,12
POINT,1.284,1.284
POINT,1.239,-1.239
TDOA,1.260,0.045
```

### 8.2 Start Live Point Cloud Visualiser

```bash
cd EchoCloud/python

# With hardware connected:
python3 echocloud_viz.py --port /dev/ttyUSB0

# Demo mode (no hardware needed — simulates a room):
python3 echocloud_viz.py --demo

# Save a data log:
python3 echocloud_viz.py --port /dev/ttyUSB0 --log run1.csv
```

### 8.3 Configuring the Firmware

All parameters live in `firmware/EchoCloud/config.h`:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `TX_BURST_US` | 500 | Burst duration — longer = more energy, more dead zone |
| `MAX_SAMPLES` | 600 | ADC capture window — larger = longer range |
| `ADC_OVERSAMPLE` | 4 | Readings averaged — higher = less noise, slower |
| `DEAD_ZONE_SAMPLES` | 20 | Skip initial samples (direct blast) |
| `SCAN_INTERVAL_MS` | 200 | Time between pings — shorter = faster but risks reverb |
| `OUTPUT_CSV` | 1 | 1=CSV output, 0=JSON output |
| `SERVO_ENABLED` | 0 | 1 to enable servo scanning |
| `DEBUG_RAW` | 0 | 1 to dump full ADC buffer (very verbose) |

### 8.4 Enable Servo Scanning

1. Set `#define SERVO_ENABLED 1` in `config.h`
2. Install the ESP32Servo library:
   ```bash
   arduino-cli lib install "ESP32Servo"
   ```
3. Connect SG90 servo to GPIO18
4. Recompile and upload

The servo will sweep 0°–180° in 5° steps, and the visualiser will track the angle.

---

## 9. Output Format

### CSV Format (default, `OUTPUT_CSV = 1`)

```
# Comment lines start with #
timestamp_ms,rx0_m,rx1_m,rx2_m,rx3_m,valid_mask
543,1.234,1.189,1.201,1.245,15
```

- `timestamp_ms` — milliseconds since boot
- `rx0_m` … `rx3_m` — distance in metres (−1.0 = no echo detected)
- `valid_mask` — bitmask: bit 0=RX0, bit 1=RX1, bit 2=RX2, bit 3=RX3

### Point Lines

```
POINT,1.284,1.284
TDOA,1.260,0.045
SERVO,45
```

- `POINT,x,y` — bearing-method point (metres, robot-centric)
- `TDOA,x,y` — TDOA-refined point
- `SERVO,deg` — current servo angle

### JSON Format (`OUTPUT_CSV = 0`)

```json
{"t":543,"rx":[1.234,1.189,1.201,1.245],"v":15}
```

---

## 10. Troubleshooting

### No Serial Output After Upload

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Nothing on serial monitor | Wrong baud rate | Set monitor to 115200 |
| Garbage characters | Wrong port or baud | Check `arduino-cli board list` |
| Sketch won't upload | ESP32 not in bootload mode | Hold BOOT button while clicking Upload, release when "Connecting..." appears |
| `Failed to connect` | CH340/CP210x driver missing | Install USB driver for your OS |

**Linux port permission:**
```bash
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### All Distances Read −1.000 (No Echoes Detected)

| Cause | Check | Fix |
|-------|-------|-----|
| Transducer not oscillating | Measure GPIO26 with multimeter/oscilloscope while running | Check transistor wiring, measure ≈ 2.5V DC (50% PWM average) |
| Receiver not amplifying | Probe op-amp output pin | Verify LM358 VCC = 5V, check gain resistors (39kΩ/1kΩ) |
| Envelope detector not conducting | Probe 1N4148 cathode | Should see voltage when near a hard surface |
| ADC reading too low | Add `Serial.printf("ADC=%d\n", analogRead(36))` | Should see values > 200 when near wall |
| Dead zone too large | `DEAD_ZONE_SAMPLES` too high | Reduce to 10 or even 5 |
| Threshold too high | Strong noise raising floor | Set `THRESHOLD_MULT` to 2 instead of 3 |

### Distances Noisy / Jumping

| Cause | Fix |
|-------|-----|
| Electrical noise on ADC | Add 100nF bypass cap from each RX pin to GND, as close to ESP32 as possible |
| ADC cross-talk between channels | Use `ADC_OVERSAMPLE 8` instead of 4 |
| Mechanical vibration | Decouple transducers from chassis with foam tape |
| Room reverberation | Add acoustic foam to robot, reduce `SCAN_INTERVAL_MS` won't help — increase it to 500ms |
| Power supply noise | Add 10µF + 100nF caps on 5V rail near LM358 |

### Distances Reading Too Short (Less Than Actual)

The ESP32 ADC measures the direct blast or electrical leakage, not the echo.

- Increase `DEAD_ZONE_SAMPLES` (try 30–50)
- Add physical shielding between TX and RX (a cardboard divider)
- Reduce `TX_BURST_US` to 250 µs (fewer cycles, shorter dead zone)

### Distances Reading Too Long (More Than Actual)

The echo detector is picking up a secondary (multi-path) reflection.

- The first echo is the correct one — it's not being detected because it's below threshold
- Reduce `THRESHOLD_MULT` to 2
- Check if receiver amplifier gain is sufficient (scope the LM358 output)

### ESP32 Resets / Crashes During Sampling

- The ADC sample loop is CPU-intensive; the watchdog may trigger
- Add `esp_task_wdt_reset()` inside the sampling loop, or disable the WDT:
  ```cpp
  #include "esp_task_wdt.h"
  esp_task_wdt_deinit();  // in setup()
  ```
- Alternatively, reduce `MAX_SAMPLES` to 400

### Python Visualiser: `ModuleNotFoundError`

```bash
pip3 install pyserial matplotlib numpy --break-system-packages
```

### Python Visualiser: No Points Appearing

1. Run `--demo` first to verify the plot works
2. Check the port: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
3. Verify the ESP32 is outputting `POINT,x,y` lines (use `screen` to check raw serial)
4. Check `POINT_ALPHA` and `FADE_SECONDS` — points may be fading before you see them

### LM358 Getting Hot

Normal if gain is very high and input is clipping. Reduce gain to 20× (R2 = 19kΩ) if the LM358 feels warm. It should run at room temperature.

---

## 11. Performance & Limitations

### What Works Well

- **Range:** 0.5–12 m on hard surfaces (walls, furniture, people)
- **Resolution:** ~3–5 cm range resolution
- **Update rate:** 5 pings/second (200 ms interval)
- **Surfaces:** Concrete, brick, wood, glass, people — anything that reflects sound
- **Environments:** Indoor rooms, hallways, corridors

### Known Limitations

| Limitation | Cause | Workaround |
|------------|-------|------------|
| Poor on soft surfaces | Foam, fabric absorb 40 kHz | No fix; stick to hard-surface environments |
| Dead zone 0–50 cm | Direct blast blinds receivers | Mount TX above receivers with baffle |
| Angular resolution ~45° per ping | Wide beam pattern of transducers | Add servo scan for 5° resolution |
| Temperature affects range | c varies with temperature | Add DHT22 and use `c = 331.3 + 0.606 × T` |
| Cross-talk between receivers | Electrical coupling | Better PCB layout / shielding |
| Cannot detect glass at shallow angles | Specular reflection away from robot | Tilt TX slightly downward |

### Sample Rate Measurement

With `DEBUG_TIMING = 1`, the firmware prints the actual measured sample period:
```
# Sample period = 212 µs  (~4717 sps)
```

Typical values on ESP32 at 240 MHz with 4 receivers and 4× oversampling:
- ~4,500–5,500 samples/s per channel
- Range resolution = 343 / (2 × 5000) ≈ 3.4 cm ✓

---

## 12. Arduino Nano Version

The Arduino Nano (ATmega328P) is a great alternative to the ESP32 when you want a **5 V system**, have one lying on your desk already, or need maximum compatibility with beginner tutorials. It runs the same cross-correlation algorithm and produces the same serial output format — the Python visualiser works unchanged.

### 12.1 Why Use the Nano?

| Reason | Detail |
|--------|--------|
| **5 V logic** | Receiver circuit is simpler — no voltage divider needed on ADC inputs |
| **Widely available** | Every electronics shop, plus dirt-cheap clones (AU$3) |
| **Familiar** | More beginner-friendly than ESP32; vast documentation |
| **Lower power** | ATmega328P draws ~15 mA active vs ESP32's ~80–200 mA |
| **Same output** | Identical serial CSV/JSON + POINT lines → same Python visualiser |

### 12.2 Nano vs ESP32 — Side-by-Side

| Feature | ESP32 | Arduino Nano |
|---------|-------|-------------|
| CPU | 240 MHz dual-core | 16 MHz single-core |
| SRAM | 520 KB | **2 KB** (critical constraint) |
| Flash | 4 MB | 32 KB |
| ADC resolution | 12-bit (0–4095) | 10-bit (0–1023) |
| ADC stored as | `uint16_t` | `uint8_t` (top 8 bits, `>> 2`) |
| Buffer per run | 4 × 600 samples = 4,800 B | 4 × 200 samples = **800 B** |
| 40 kHz generation | LEDC PWM (any pin) | Timer1 OC1A (**pin 9 only**) |
| TX pin | GPIO26 | **Pin 9** (fixed by hardware) |
| Voltage logic | 3.3 V | **5 V** |
| Voltage divider on RX | Required | **Not needed** |
| Float printing | `Serial.printf()` | `dtostrf()` + `Serial.print()` |
| Max range | ~15 m | ~12 m |
| Update rate | ~5 Hz | ~4 Hz |
| Range resolution | ~3.4 cm | ~7.7 cm |
| Cost (clone) | AU$5.50 | AU$3.50 |

### 12.3 Nano Firmware Files

```
firmware/EchoCloud_Nano/
├── EchoCloud_Nano.ino    ← Main sketch (Timer1 burst + loop)
├── config_nano.h         ← All pin/parameter constants (Nano-specific)
├── sonar_nano.h          ← API: structs + prototypes
└── sonar_nano.cpp        ← Implementation: ping, ADC, cross-correlation, output
```

### 12.4 The 2 KB RAM Challenge — How We Solved It

The ATmega328P has only 2048 bytes of SRAM. The buffer alone (`NUM_RX × MAX_SAMPLES × bytes_per_sample`) must fit with headroom for the stack and Serial buffers (~300 bytes).

```
  Available SRAM: 2048 bytes
  ├── Serial TX buffer:   64 bytes
  ├── Serial RX buffer:   64 bytes
  ├── Arduino framework: ~100 bytes
  ├── Stack (locals):    ~150 bytes
  └── Our buffer:        800 bytes  ← tight but safe
       4 receivers × 200 samples × 1 byte = 800 bytes  ✓

  Key trick: store ADC reading as uint8_t, not uint16_t
  → analogRead() >> 2   (top 8 bits of 10-bit result)
  → 2 KB used instead of 4 KB
  → saves 800 bytes  ✓

  Confirmed by compiler:
  Global variables use 1084 bytes (52%) leaving 964 bytes for locals.
```

All string literals use `F()` macro (stores strings in Flash, not SRAM). No `sprintf` or `printf` (heap-heavy on AVR) — uses `dtostrf()` for float-to-string.

### 12.5 40 kHz Generation — Timer1 Hardware PWM

The ESP32 uses its LEDC peripheral (available on any pin). The Nano must use **Timer1**, and its output is hardware-wired to **pin 9** (OC1A). You cannot change this.

```
  Timer1 Fast PWM configuration:
  ─────────────────────────────────────────────────────────
  Register  Value    Meaning
  ICR1      399      TOP = 399  →  f = 16 MHz / 400 = 40,000 Hz exactly
  OCR1A     200      Compare = 200/399 ≈ 50 % duty cycle
  TCCR1A    0xA2     COM1A1=1 (clear OC1A on match), WGM11=1
  TCCR1B    0x18     WGM13=1, WGM12=1 (Fast PWM mode 14)
  CS1x      001      No prescaler  (only set during burst)
  ─────────────────────────────────────────────────────────

  To START burst:    TCCR1B |=  (1 << CS10)
  To STOP burst:     TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10))
                     digitalWrite(9, LOW)
```

```
  ASCII DIAGRAM — Timer1 waveform on pin 9:

  Burst (500 µs = 20 cycles):
  ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐ ┌─┐
  ┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └─┘ └────
  |← 25 µs →|← 25 µs →|  …                |
  |←────────── 500 µs total ───────────────|

  After burst: pin held LOW, Timer1 stopped.
```

### 12.6 Nano Circuit Changes

The Nano runs at **5 V**, which simplifies the receiver circuit in two ways:

1. **No voltage divider** on the ADC input — the Nano's ADC reference is AVcc = 5 V, matching the LM358's output swing.
2. **LM358 and transducers** can share the single 5 V supply from the Nano's `5V` pin.

#### Simplified Receiver Circuit (Nano)

```
  ESP32 version needed this at ADC end:
    LM358 OUT → 1N4148 → 10nF → [10kΩ/10kΩ divider] → ADC
                                  ↑ required to halve 5V to 2.5V

  Nano version — remove the divider:
    LM358 OUT → 1N4148 → 10nF → ADC pin  ✓  (5V ADC, no divider needed)
```

The transmitter circuit is **identical** to the ESP32 version, except pin 9 is used instead of GPIO26.

#### Nano Complete Signal Chain

```
  ┌─ TX CHAIN ───────────────────────────────────────────────┐
  │                                                          │
  │  Nano pin 9   2N2222     MA40S4S                         │
  │  (OC1A) ─[100Ω]─[B]─[C]─[100Ω]─[TX+]─[TX−]─GND       │
  │                    [E]─GND                               │
  │                  5V (Nano 5V pin)                        │
  └──────────────────────────────────────────────────────────┘

  ┌─ RX CHAIN (×4, Nano — NO voltage divider) ───────────────┐
  │                                                          │
  │  MA40S4R  Coupling   Amp (40×)   Rectifier    Nano ADC   │
  │  [RX+]─[0.1µF]─[LM358]─[1N4148]─[10nF─GND]─── A0..A3  │
  │  [RX−]─GND      │                                       │
  │               [1MΩ─5V]   [39k/1k feedback]              │
  └──────────────────────────────────────────────────────────┘

  Power: all 5V from Nano 5V pin. No 3.3V rails used.
```

### 12.7 Nano Wiring Guide

```
POWER:
  USB-A → micro-USB cable to Nano
  Nano 5V pin → LM358 VCC (×2 chips)
  Nano 5V pin → 2N2222 collector rail
  Nano GND    → common ground (all components)

TRANSMITTER:
  Nano pin 9  → 100Ω → 2N2222 base
  2N2222 emitter → GND
  2N2222 collector → 100Ω → MA40S4S (+)
  MA40S4S (−) → GND

RECEIVER 0 (A0):
  MA40S4R(0) (+) → 0.1µF → LM358(A) pin3 (+IN)
  MA40S4R(0) (−) → GND
  1MΩ between LM358(A) pin3 and 5V
  LM358(A) pin2 → 39kΩ/1kΩ divider to GND  (feedback)
  LM358(A) pin1 → 1kΩ → 1N4148 anode
  1N4148 cathode → node A
  Node A → 10nF → GND
  Node A → Nano A0               ← no voltage divider!

  Repeat for RX1→A1, RX2→A2, RX3→A3.

OPTIONAL STATUS LED:
  Nano pin 13 (built-in) — already connected on board.
```

### 12.8 Installation & Compile (Nano)

```bash
# arduino-cli already installed (see Section 7.2)
# AVR core:
arduino-cli core install arduino:avr

# Compile for clone Nano (CH340, old bootloader — most common):
arduino-cli compile \
  --fqbn arduino:avr:nano:cpu=atmega328old \
  firmware/EchoCloud_Nano

# Compile for official Arduino Nano (new bootloader):
arduino-cli compile \
  --fqbn arduino:avr:nano:cpu=atmega328 \
  firmware/EchoCloud_Nano

# Expected output (both variants):
# Sketch uses 7274 bytes (23%) of program storage space.
# Global variables use 1084 bytes (52%) of dynamic memory,
#   leaving 964 bytes for local variables.

# Upload (clone Nano, adjust port as needed):
arduino-cli upload \
  --fqbn arduino:avr:nano:cpu=atmega328old \
  --port /dev/ttyUSB0 \
  firmware/EchoCloud_Nano
```

> **Which FQBN do I use?** If you bought the Nano from AliExpress/eBay for under AU$5, it almost certainly has a CH340G USB chip and old bootloader — use `cpu=atmega328old`. Official Arduino-branded Nanos bought from reputable retailers use the new bootloader — use `cpu=atmega328`.

### 12.9 Running with the Python Visualiser

The Nano outputs **identical** CSV and POINT lines to the ESP32. No changes to the Python script are needed:

```bash
# Nano connected — same command as ESP32:
python3 python/echocloud_viz.py --port /dev/ttyUSB0

# Or demo mode (no hardware):
python3 python/echocloud_viz.py --demo
```

### 12.10 Nano-Specific Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Upload fails immediately | Wrong bootloader variant | Try `cpu=atmega328old` if using `cpu=atmega328`, or vice versa |
| Upload fails with CH340 on Linux | Driver / permission | `sudo usermod -a -G dialout $USER` then log out/in |
| Upload fails with CH340 on Windows | CH340 driver not installed | Install CH340 driver from WCH website |
| `avrdude: stk500_recv()` error | Nano not resetting on upload | Press reset button as upload starts, or add 100nF cap between DTR and RESET pin |
| All distances −1.000 | ADC seeing no envelope signal | Without 5V bias, LM358 output may be near 0 V. Check 1MΩ bias resistor is connected to 5V, not 3.3V |
| Distances very noisy | ATmega ADC more sensitive to supply noise than ESP32 | Add 100nF cap between AREF pin and GND; add 10µF on 5V rail near chip |
| Sketch too large (> 30,720 B) | Only possible if adding many libraries | Disable Serial debugging, remove unused features |
| `Low memory, stability problems` warning | Global RAM > 75% | Already at 52% — safe. Warning triggers at ~80% |
| Timer1 conflict | If using a servo library simultaneously | `Servo.h` on AVR uses Timer1, conflicting with our 40 kHz burst. Use `SoftwareServo` or `VarSpeedServo` which use Timer2 instead |

### 12.11 Nano Performance Summary

```
  Measured on Nano clone (CH340, ATmega328P @ 16 MHz):

  Sample period:   ~448 µs  (4 channels × ~112 µs analogRead)
  Sample rate:     ~2,232 sps per channel
  Max range:       ~12.1 m  (200 samples × 448µs × 343/2)
  Range resolution: ~7.7 cm  (343 × 448µs / 2)
  Update rate:     ~4 Hz    (250 ms scan interval)
  Flash used:      7,274 B  (23% of 30,720 B)
  RAM used:        1,084 B  (52% of 2,048 B)
  Free RAM:        964 B    (safe — stack fits comfortably)
```

For most indoor robot navigation (avoiding walls, furniture, doorways) at up to 5 m range, the Nano version is entirely adequate.

---

## 13. Future Improvements

### Easy Additions

1. **Temperature compensation** — Wire a DHT22 to GPIO5; measure temperature; use `c = 331.3 + 0.606 × T_celsius`

2. **3D scanning** — Add a second SG90 servo for tilt. Change `sonar_print_points()` to output `POINT3D,x,y,z`

3. **Chirp instead of tone** — Replace the fixed-frequency burst with a linear frequency chirp (e.g., 35–45 kHz). Pulse compression via matched filtering gives better range resolution in noise.

4. **WiFi streaming** — Enable the ESP32 WiFi and stream JSON over UDP to a laptop. Removes the serial cable.

5. **OLED display** — Wire a 0.96" SSD1306 to I²C (GPIO21/22); show the four distances in real time.

6. **Occupancy grid** — Python side: maintain a 2D array representing occupied/free space; mark cells hit by POINT lines. Visualise with `matplotlib.imshow`.

7. **Obstacle avoidance** — If the robot has L298N motor driver, stop when any distance < 0.3 m.

### Harder Additions

8. **TDOA array processing** — With 6+ receivers and precise timing, implement a full TDOA least-squares solver for accurate direction without a servo.

9. **FIR bandpass filter** — Implement a 40 kHz bandpass FIR filter on the raw ADC data (before envelope detection in software). Improves SNR in noisy environments.

10. **ROS2 bridge** — Publish `sensor_msgs/LaserScan` messages from the Python script for full ROS2 navigation stack compatibility.

---

## 14. Theory References

| Topic | Source |
|-------|--------|
| Cross-correlation for echo ranging | *DSP using MATLAB*, Ingle & Proakis, Chapter 2 (p. 65) |
| Echo / reverberation effects | Same book, Chapter 1 (pp. 18–20) |
| Chirp signals & pulse compression | Same book, Chapters 5 & 12 |
| FIR filter design | Same book, Chapter 7 |
| Spread-spectrum / PN sequences | Same book, Chapter 12.8 |
| DTMF tone generation (Goertzel) | Same book, Chapter 12.6 |
| Acoustic wave propagation | *Fundamentals of Acoustics*, Kinsler et al. |
| ESP32 LEDC peripheral | Espressif ESP32 Technical Reference Manual, Chapter 13 |
| ESP32 ADC characteristics | Espressif ESP32 Technical Reference Manual, Chapter 29 |

---

## License

MIT License — free to use, modify, and share.

**Build credits:** Concept developed from a DSP textbook study session tracing ideas from DTMF tone generation → acoustic radar → multi-receiver sonar → robot navigation.

---

*Ready to give your robot bat-like superpowers? Wire it up, flash it, and watch the walls appear.*
