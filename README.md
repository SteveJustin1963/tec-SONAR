# tec-SONAR
## TEC-1 Sonar Experiments — Distance, Direction & Object Tracking in C

> **Sound goes out. Echo comes back. Measure the gap — and you know where everything is.**
> Foundational sonar algorithms in plain C: time-of-flight ranging, triangle-geometry direction-finding, and object tracking.

---

## Table of Contents

1. [Overview](#1-overview)
2. [How Sonar Works — Theory](#2-how-sonar-works--theory)
3. [Programs](#3-programs)
   - 3.1 [distance1.c — Basic Time-of-Flight](#31-distance1c--basic-time-of-flight)
   - 3.2 [distance2.c — Compact ToF](#32-distance2c--compact-tof)
   - 3.3 [direction1.c — Triangle Direction-Finding](#33-direction1c--triangle-direction-finding)
   - 3.4 [track.c — Object Tracking](#34-trackc--object-tracking)
4. [Building & Running](#4-building--running)
5. [Sonar Facts & Notes](#5-sonar-facts--notes)
6. [References](#6-references)

---

## 1. Overview

These programs explore the fundamental maths behind sonar sensing — the same principles used in submarine ASDIC, medical ultrasound, and cheap HC-SR04 rangefinder modules.

The four programs form a progression:

```
distance1/2.c     →    direction1.c    →    track.c
  "How far?"           "Which way?"       "Where is it going?"
```

All programs are written in standard C (C99) with no external dependencies beyond `<math.h>`.

---

## 2. How Sonar Works — Theory

### 2.1 Time of Flight (ToF)

A sonar device emits a sound pulse and listens for the echo.  The round-trip time gives distance:

```
distance = (speed_of_sound × time) / 2
```

The `/2` accounts for the pulse travelling *out and back*.

| Medium       | Speed of sound |
|--------------|---------------|
| Air (20 °C)  | ~343 m/s      |
| Water        | ~1480 m/s     |
| Steel        | ~5960 m/s     |

These programs assume **340 m/s** (a common approximation for room-temperature air).

**Example:**
- Echo returns after 5.88 ms → distance = (340 × 0.00588) / 2 ≈ **1.0 m**

---

### 2.2 Direction-Finding via Triangulation

When *multiple* sensors receive the same echo at slightly different times, the geometry of the resulting triangle reveals the bearing of the target.

```
        Sensor A
        /\
       /  \   sides a, b, c = distances from each sensor to target
      /    \       and baseline between sensors
     /      \
Sensor B----Target (C)
```

Given sides `a`, `b`, `c`, the **law of cosines** classifies the angle at the target:

```
cos(C) = (a² + b² - c²) / (2ab)
```

- `a² + b² > c²`  →  acute triangle  (target broadly ahead)
- `a² + b² = c²`  →  right triangle  (target at 90°)
- `a² + b² < c²`  →  obtuse triangle (target behind the baseline)

---

### 2.3 Object Tracking

Once the distance `d` to a moving object is known, and its velocity `v` is measured (e.g., by Doppler shift or successive ranging), the time-to-intercept is:

```
t = (2 × d) / v
```

---

## 3. Programs

### 3.1 `distance1.c` — Basic Time-of-Flight

Prompts for the round-trip echo time and computes distance at 340 m/s.

```
Enter the time it took for the sound waves to bounce back: 0.00588
The distance to the object is 1.00 meters.
```

**Formula used:**
```c
distance = (speedOfSound * time) / 2.0;   // speedOfSound = 340.0 m/s
```

---

### 3.2 `distance2.c` — Compact ToF

Functionally identical to `distance1.c` but stripped to minimal form — useful as a reference or embedded snippet.

---

### 3.3 `direction1.c` — Triangle Direction-Finding

Takes three integer side lengths (sonar range measurements) and classifies the triangle formed by those distances.

```
Enter the first side of the triangle:   3
Enter the second side of the triangle:  4
Enter the third side of the triangle:   5
The triangle is a right triangle.
```

**Algorithm:**

1. Validate the triangle inequality (`a+b > c` etc.).
2. Rotate sides so `c` is always the longest (largest interior angle opposite `c`).
3. Compare `a² + b²` vs `c²` using integer arithmetic (avoids floating-point rounding).

```c
if (a2 + b2 == c2)   printf("right triangle\n");
else if (a2 + b2 > c2) printf("acute triangle\n");
else                   printf("obtuse triangle\n");
```

**Bug fixed:** Original code had `and(i<=90)` (syntax error) and used an incorrect angle estimate derived from the inradius formula. Replaced with exact integer law-of-cosines classification using `long long` to prevent overflow on large inputs.

---

### 3.4 `track.c` — Object Tracking

Given an object's velocity and current distance, calculates time-to-intercept.

```
Enter the velocity (in m/s) of the object: 10
Enter the distance (in m) of the object:   50
The object is 50 m away.
It will take 10.00 seconds for the object to reach the observer.
```

**Bug fixed:** Original used integer division `(2 * d) / v` before assignment to `float t`, losing precision. Fixed to `(2.0f * d) / v`. Also corrected `printf("%.2f", d)` (undefined behaviour — `d` is `int`) to `printf("%d", d)`.

---

## 4. Building & Running

Requires a C99 compiler with `libm`. On Linux/Mac:

```bash
# Compile
gcc -std=c99 -Wall -o distance1  distance1.c  -lm
gcc -std=c99 -Wall -o distance2  distance2.c  -lm
gcc -std=c99 -Wall -o direction1 direction1.c -lm
gcc -std=c99 -Wall -o track      track.c      -lm

# Run
./distance1
./direction1
```

On Windows (MinGW):
```bash
gcc -std=c99 -Wall -o direction1.exe direction1.c -lm
```

---

## 5. Sonar Facts & Notes

- Sonar frequencies range from infrasonic to ultrasonic — navy systems reach **120,000 Hz**, well above human hearing (~20 kHz upper limit).
- Sound propagates much better through water than air, which is why submarines can hear sonar pings from great distances.
- Navy sonar systems generate pulses up to **235 dB** — the loudest rock concert peaks at ~130 dB.
- Sound at 235 dB can travel hundreds of miles underwater and retain ~140 dB at 300 miles.
- In-air sonar sensors (e.g., HC-SR04) operate on the same ToF principle but at short range (2 cm – 4 m).
- Passive sonar only listens (no pulse transmitted) — used by submarines to avoid detection while still ranging.
- Aircraft use dropped **sonobuoys** for active sonar patrol without the aircraft emitting any signal.

---

## 6. References

- Sonar theory overview: [YouTube — How SONAR Works](https://www.youtube.com/watch?v=z4uxC7ISd-c)
- Multi-receiver acoustic array hardware: [aisler.net — bitluni array8integrated](https://aisler.net/bitluni/archive/array8integrated)
- Acoustic point-cloud shader demo: [shadertoy.com/view/NdXfDl](https://www.shadertoy.com/view/NdXfDl)
- Full SonarScanner firmware reference: [github.com/bitluni/SonarScannerV1](https://github.com/bitluni/SonarScannerV1)
- EchoCloud (multi-receiver ESP32 build in this repo): [`EchoCloud/README.md`](EchoCloud/README.md)
