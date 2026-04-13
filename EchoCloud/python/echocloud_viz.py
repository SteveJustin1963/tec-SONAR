#!/usr/bin/env python3
"""
EchoCloud — Real-Time Point-Cloud Visualiser
Reads CSV / POINT / TDOA lines from ESP32 serial port and
renders a live 2D scatter plot in matplotlib.

ASCII FLOWCHART — Visualiser Logic
===================================

  start
    │
    ▼
  parse_args()   ← --port /dev/ttyUSB0  --baud 115200
    │              --log  data.csv  --demo (offline sim)
    │
    ▼
  open_serial()  OR  demo_mode()
    │
    ▼
  setup_plot()   ← matplotlib interactive figure
    │
    ╔══════════════════════════════╗
    ║         MAIN LOOP            ║
    ║                              ║
    ║  read_line_from_serial()     ║
    ║        │                     ║
    ║   ┌────▼────────────────┐    ║
    ║   │  classify line      │    ║
    ║   │  "# …"  → comment   │    ║
    ║   │  "1234,…" → CSV row │    ║
    ║   │  "POINT,x,y" → pt   │    ║
    ║   │  "TDOA,x,y"  → tdoa │    ║
    ║   │  "SERVO,deg" → angle│    ║
    ║   └────────────────────┘    ║
    ║        │                     ║
    ║   ┌────▼─────────────────┐   ║
    ║   │ accumulate points   │   ║
    ║   │ (ring buffer, 500)  │   ║
    ║   └────┬─────────────────┘   ║
    ║        │                     ║
    ║   every REFRESH_HZ frames:   ║
    ║   update_plot(scatter)       ║
    ║        │                     ║
    ╚════════╪═════════════════════╝
             │
           Ctrl+C → save log → exit

Usage:
    python3 echocloud_viz.py --port /dev/ttyUSB0
    python3 echocloud_viz.py --demo            # offline simulation
    python3 echocloud_viz.py --port /dev/ttyUSB0 --log run1.csv
"""

import argparse
import sys
import time
import math
import random
import threading
import collections
import csv

import serial
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import FancyArrowPatch

# ── Configuration ──────────────────────────────────────────────
BAUD_RATE      = 115200
MAX_POINTS     = 500        # Ring-buffer size for scatter plot
REFRESH_HZ     = 10         # Plot update frequency
ROBOT_RADIUS   = 0.10       # Metres, for drawing robot body
GRID_SIZE      = 10         # ± metres shown on plot
POINT_ALPHA    = 0.7
TDOA_COLOUR    = 'red'
BEARING_COLOUR = 'cyan'
FADE_SECONDS   = 5.0        # Points fade out after this many seconds

# ── Data containers ────────────────────────────────────────────
class PointRecord:
    __slots__ = ('x', 'y', 't', 'kind')
    def __init__(self, x, y, kind='bearing'):
        self.x    = x
        self.y    = y
        self.t    = time.monotonic()
        self.kind = kind  # 'bearing' | 'tdoa'

point_buffer = collections.deque(maxlen=MAX_POINTS)
latest_distances = [None] * 4
latest_valid     = 0
servo_angle      = 0
log_rows         = []
lock             = threading.Lock()

# ── Serial reader thread ───────────────────────────────────────
def serial_reader(ser):
    """Background thread: parse lines from serial and populate buffers."""
    global servo_angle, latest_valid
    while True:
        try:
            raw = ser.readline()
        except serial.SerialException:
            print("[ERROR] Serial disconnected.", file=sys.stderr)
            break

        if not raw:
            continue

        try:
            line = raw.decode('ascii', errors='replace').strip()
        except Exception:
            continue

        if not line or line.startswith('#'):
            continue

        with lock:
            # ── CSV distance row: timestamp,d0,d1,d2,d3,valid_mask ──
            if line[0].isdigit():
                parts = line.split(',')
                if len(parts) >= 5:
                    try:
                        latest_distances[:] = [float(p) for p in parts[1:5]]
                        latest_valid = int(parts[5]) if len(parts) > 5 else 0
                        log_rows.append(parts)
                    except ValueError:
                        pass

            # ── POINT,x,y ────────────────────────────────────────
            elif line.startswith('POINT,'):
                parts = line.split(',')
                if len(parts) == 3:
                    try:
                        point_buffer.append(
                            PointRecord(float(parts[1]), float(parts[2]),
                                        'bearing'))
                    except ValueError:
                        pass

            # ── TDOA,x,y ─────────────────────────────────────────
            elif line.startswith('TDOA,'):
                parts = line.split(',')
                if len(parts) == 3:
                    try:
                        point_buffer.append(
                            PointRecord(float(parts[1]), float(parts[2]),
                                        'tdoa'))
                    except ValueError:
                        pass

            # ── SERVO,degrees ────────────────────────────────────
            elif line.startswith('SERVO,'):
                try:
                    servo_angle = int(line.split(',')[1])
                except (ValueError, IndexError):
                    pass


# ── Demo generator (runs without real hardware) ───────────────
def demo_generator():
    """Simulate a rotating sonar head scanning a rectangular room."""
    global servo_angle
    angle = 0.0
    room_walls = [
        # (x_start, y_start, x_end, y_end)
        (-3, -2,  3, -2),   # South wall
        ( 3, -2,  3,  2),   # East wall
        ( 3,  2, -3,  2),   # North wall
        (-3,  2, -3, -2),   # West wall
    ]

    def ray_wall_intersect(ox, oy, dx, dy, walls):
        """Ray–segment intersection; returns closest hit distance."""
        min_t = 20.0
        for wx0, wy0, wx1, wy1 in walls:
            # Parametric: P = O + t*D,  Q = W0 + u*(W1-W0)
            denom = dx*(wy1-wy0) - dy*(wx1-wx0)
            if abs(denom) < 1e-9:
                continue
            t = ((wx0-ox)*(wy1-wy0) - (wy0-oy)*(wx1-wx0)) / denom
            u = ((wx0-ox)*dy      - (wy0-oy)*dx)           / denom
            if 0.0 <= t <= min_t and 0.0 <= u <= 1.0:
                min_t = t
        return min_t

    while True:
        for a_deg in range(0, 360, 5):
            angle = math.radians(a_deg)
            offsets = [0.0, 0.15, -0.15, 0.30]
            for off in offsets:
                a = angle + off
                dx, dy = math.cos(a), math.sin(a)
                dist = ray_wall_intersect(0, 0, dx, dy, room_walls)
                dist += random.gauss(0, 0.03)   # Noise
                x = dist * dx
                y = dist * dy
                with lock:
                    point_buffer.append(PointRecord(x, y, 'bearing'))
            servo_angle = a_deg
            time.sleep(0.05)


# ── Matplotlib setup ───────────────────────────────────────────
def setup_plot():
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_facecolor('#1a1a2e')
    fig.patch.set_facecolor('#0f0f23')
    ax.set_xlim(-GRID_SIZE, GRID_SIZE)
    ax.set_ylim(-GRID_SIZE, GRID_SIZE)
    ax.set_xlabel('X (m) — Forward →', color='white')
    ax.set_ylabel('Y (m) — Left →',    color='white')
    ax.set_title('EchoCloud — Live Acoustic Point Cloud', color='cyan', fontsize=13)
    ax.tick_params(colors='white')
    for spine in ax.spines.values():
        spine.set_edgecolor('#444')
    ax.grid(True, color='#333', linestyle='--', alpha=0.5)
    ax.set_aspect('equal')

    # Robot body marker
    robot_circle = plt.Circle((0, 0), ROBOT_RADIUS,
                               color='#00ff88', fill=False, linewidth=2)
    ax.add_patch(robot_circle)
    forward_arrow = FancyArrowPatch(
        (0, 0), (ROBOT_RADIUS * 2, 0),
        arrowstyle='->', color='#00ff88', linewidth=2)
    ax.add_patch(forward_arrow)
    ax.text(0.02, ROBOT_RADIUS * 2.5, 'FRONT',
            color='#00ff88', fontsize=7, ha='center')

    # Scatter: bearing points (cyan) and TDOA points (red)
    sc_bearing = ax.scatter([], [], s=15, c=BEARING_COLOUR,
                             alpha=POINT_ALPHA, label='Bearing pts')
    sc_tdoa    = ax.scatter([], [], s=40, c=TDOA_COLOUR,
                             marker='x', linewidths=2, label='TDOA pts')

    # Distance text panel
    dist_text = ax.text(
        -GRID_SIZE + 0.3, GRID_SIZE - 0.6,
        '', color='white', fontsize=9,
        fontfamily='monospace',
        verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='#111', alpha=0.7))

    ax.legend(loc='lower right', facecolor='#222', labelcolor='white',
              fontsize=8)

    return fig, ax, sc_bearing, sc_tdoa, dist_text


def update_plot(frame, sc_bearing, sc_tdoa, dist_text):
    now = time.monotonic()
    with lock:
        pts = list(point_buffer)
        dists = list(latest_distances)
        valid = latest_valid

    # Age-based filtering
    bearing_x, bearing_y = [], []
    tdoa_x,    tdoa_y    = [], []

    for p in pts:
        age = now - p.t
        if age > FADE_SECONDS:
            continue
        if p.kind == 'tdoa':
            tdoa_x.append(p.x); tdoa_y.append(p.y)
        else:
            bearing_x.append(p.x); bearing_y.append(p.y)

    sc_bearing.set_offsets(
        np.column_stack([bearing_x, bearing_y]) if bearing_x
        else np.empty((0, 2)))
    sc_tdoa.set_offsets(
        np.column_stack([tdoa_x, tdoa_y]) if tdoa_x
        else np.empty((0, 2)))

    # Distance readout panel
    lines = ['Distances (m):']
    labels = ['RX0 FL', 'RX1 FR', 'RX2 RL', 'RX3 RR']
    for ch in range(4):
        bit = 1 << ch
        ok = '●' if (valid & bit) else '○'
        val = f'{dists[ch]:.2f}' if dists[ch] is not None else '---'
        lines.append(f'  {ok} {labels[ch]}: {val} m')
    lines.append(f'Servo: {servo_angle}°')
    dist_text.set_text('\n'.join(lines))

    return sc_bearing, sc_tdoa, dist_text


# ── Main ───────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='EchoCloud — Acoustic Point-Cloud Visualiser')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=BAUD_RATE,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--log',  default=None,
                        help='Save all distance rows to CSV file')
    parser.add_argument('--demo', action='store_true',
                        help='Run with simulated data (no hardware needed)')
    args = parser.parse_args()

    if args.demo:
        print('[INFO] Demo mode — simulated room scan')
        demo_thread = threading.Thread(target=demo_generator, daemon=True)
        demo_thread.start()
    else:
        print(f'[INFO] Opening {args.port} @ {args.baud} baud…')
        try:
            ser = serial.Serial(args.port, args.baud, timeout=1.0)
        except serial.SerialException as e:
            print(f'[ERROR] Cannot open port: {e}', file=sys.stderr)
            print('[TIP]  Try:  python3 echocloud_viz.py --demo', file=sys.stderr)
            sys.exit(1)
        reader_thread = threading.Thread(target=serial_reader,
                                         args=(ser,), daemon=True)
        reader_thread.start()

    fig, ax, sc_bearing, sc_tdoa, dist_text = setup_plot()

    ani = animation.FuncAnimation(
        fig,
        update_plot,
        fargs=(sc_bearing, sc_tdoa, dist_text),
        interval=int(1000 / REFRESH_HZ),
        blit=False,
        cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if args.log and log_rows:
            with open(args.log, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['timestamp_ms',
                             'rx0_m', 'rx1_m', 'rx2_m', 'rx3_m',
                             'valid_mask'])
                w.writerows(log_rows)
            print(f'[INFO] Saved {len(log_rows)} rows to {args.log}')


if __name__ == '__main__':
    main()
