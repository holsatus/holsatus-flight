#!/usr/bin/env python3
"""Live plotter for stm32f405-dev USB orientation CSV stream.

Expected CSV format from firmware:
    t_ms,roll,pitch,yaw,qw,qx,qy,qz
"""

import argparse
import collections
import sys
import time

import matplotlib.pyplot as plt
import serial


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live plot orientation stream over USB serial")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (CDC often ignores this)")
    parser.add_argument("--window", type=float, default=15.0, help="Visible history window in seconds")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout in seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    # Use a bounded history to keep plotting stable over long runs.
    max_points = max(200, int(args.window * 100))
    ts = collections.deque(maxlen=max_points)
    roll = collections.deque(maxlen=max_points)
    pitch = collections.deque(maxlen=max_points)
    yaw = collections.deque(maxlen=max_points)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Failed to open serial port: {exc}", file=sys.stderr)
        return 1

    print(f"Connected to {args.port} @ {args.baud}")
    print("Waiting for CSV stream...")

    plt.ion()
    fig, ax = plt.subplots(num="Orientation Stream")
    line_r, = ax.plot([], [], label="roll")
    line_p, = ax.plot([], [], label="pitch")
    line_y, = ax.plot([], [], label="yaw")
    ax.legend(loc="upper left")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("rad")
    ax.grid(True, alpha=0.3)

    t0_ms = None
    last_draw = 0.0

    try:
        while True:
            raw = ser.readline()
            if not raw:
                plt.pause(0.001)
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue
            if line.startswith("t_ms"):
                continue

            parts = line.split(",")
            if len(parts) < 4:
                continue

            try:
                t_ms = float(parts[0])
                r = float(parts[1])
                p = float(parts[2])
                y = float(parts[3])
            except ValueError:
                continue

            if t0_ms is None:
                t0_ms = t_ms

            t_s = (t_ms - t0_ms) / 1000.0
            ts.append(t_s)
            roll.append(r)
            pitch.append(p)
            yaw.append(y)

            now = time.monotonic()
            if now - last_draw < 0.03:
                continue
            last_draw = now

            line_r.set_data(ts, roll)
            line_p.set_data(ts, pitch)
            line_y.set_data(ts, yaw)

            if ts:
                x_max = ts[-1]
                x_min = max(0.0, x_max - args.window)
                ax.set_xlim(x_min, x_max + 0.1)

            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
            fig.canvas.draw_idle()
            plt.pause(0.001)

    except KeyboardInterrupt:
        print("\nStopping plotter")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
