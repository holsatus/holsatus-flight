#!/usr/bin/env python3
"""
Phase 1 HIL host: inject a synthetic IMU stream over the USART1 link and
read back the FC's AHRS_ATTITUDE_Q debug frame. Lockstep -- one SensorFrame
in, one AttitudeFrame reply out, per references/hil_implementation_plan.md.

Wire protocol (must match src/bin/hil.rs SENSOR_FRAME_*/ATTITUDE_FRAME_*):

  SensorFrame (host -> FC, 32 bytes, little-endian):
    u8  sync=0xA5, u8 type=0x01, u32 seq, f32 acc[3], f32 gyr[3], u16 crc16

  AttitudeFrame (FC -> host, 24 bytes, little-endian):
    u8  sync=0x5A, u8 type=0x02, u32 seq, f32 q[4] (i,j,k,w), u16 crc16

CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over all preceding
bytes in the frame -- matches hil.rs::crc16_ccitt exactly.

Usage:
    python3 hil_phase1_imu_inject.py /dev/tty.usbserial-XXXX static
    python3 hil_phase1_imu_inject.py /dev/tty.usbserial-XXXX tilt --roll-deg 30
    python3 hil_phase1_imu_inject.py /dev/tty.usbserial-XXXX dynamic --roll-rate 0.5
"""
import argparse
import math
import struct
import time

BAUD = 115_200  # only rate with a genuine matched-baud Phase 0 measurement;
# the "2,000,000" Phase 0 result was a mismatch artefact -- hil_echo.rs never
# set cfg.baudrate, so firmware stayed pinned at embassy's 115200 default
# while the host thought it was running at 2M. See hil.rs HIL_LINK_BAUD.
SENSOR_FRAME_LEN = 32
ATTITUDE_FRAME_LEN = 24
GRAVITY = 9.81


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_sensor_frame(seq: int, acc: tuple, gyr: tuple) -> bytes:
    body = struct.pack("<BBI3f3f", 0xA5, 0x01, seq, *acc, *gyr)
    assert len(body) == SENSOR_FRAME_LEN - 2
    return body + struct.pack("<H", crc16_ccitt(body))


def parse_attitude_frame(buf: bytes):
    if len(buf) != ATTITUDE_FRAME_LEN or buf[0] != 0x5A or buf[1] != 0x02:
        return None
    body, crc_rx = buf[:22], struct.unpack("<H", buf[22:24])[0]
    if crc16_ccitt(body) != crc_rx:
        return None
    seq, i, j, k, w = struct.unpack("<Iffff", buf[2:22])
    return {"seq": seq, "quat_ijkw": (i, j, k, w)}


def quat_to_euler_deg(i, j, k, w):
    # Aerospace roll-pitch-yaw (ZYX intrinsic) extraction. Sign/axis
    # convention should match nalgebra's UnitQuaternion::euler_angles() --
    # sanity-check against a known static test before trusting magnitudes.
    roll = math.atan2(2 * (w * i + j * k), 1 - 2 * (i * i + j * j))
    sinp = max(-1.0, min(1.0, 2 * (w * j - k * i)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * k + i * j), 1 - 2 * (j * j + k * k))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def run(ser, acc_fn, gyr_fn, duration_s: float, label: str):
    print(f"-- {label} ({duration_s:.1f}s) --")
    t0 = time.perf_counter()
    seq = 0
    last_print = 0.0
    n_bad = 0
    while time.perf_counter() - t0 < duration_s:
        t = time.perf_counter() - t0
        ser.write(build_sensor_frame(seq, acc_fn(t), gyr_fn(t)))
        reply = ser.read(ATTITUDE_FRAME_LEN)
        att = parse_attitude_frame(reply)
        if att is None:
            n_bad += 1
        elif t - last_print > 0.2:
            last_print = t
            r, p, y = quat_to_euler_deg(*att["quat_ijkw"])
            print(f"t={t:5.2f}s seq={seq:6d} roll={r:6.2f} pitch={p:6.2f} yaw={y:6.2f}")
        seq += 1
    print(f"done: {seq} sent, {n_bad} bad/missing replies\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("mode", choices=["static", "tilt", "dynamic"])
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--roll-deg", type=float, default=30.0, help="tilt mode: commanded roll angle")
    ap.add_argument("--roll-rate", type=float, default=0.5, help="dynamic mode: gyr_x rad/s")
    ap.add_argument("--warmup", type=float, default=10.0,
                     help="seconds of level/stationary data sent before the requested mode, "
                          "to carry imu_cal::apply()'s gate (1s hold + 5s hands-off + 2s "
                          "capture, ~8s) through on genuinely level data. See imu_cal.rs: the "
                          "bias capture doesn't re-check level, so a gap here (e.g. between "
                          "separate script runs) can let it consume whatever the *next* run "
                          "happens to send -- D000077-style bad bias baked in.")
    ap.add_argument("--skip-warmup", action="store_true",
                     help="skip the warm-up (only safe if calibration already completed on "
                          "level data in the current boot session, e.g. right after a prior "
                          "static/warm-up run with no idle gap since)")
    args = ap.parse_args()

    import serial
    ser = serial.Serial(args.port, args.baud, timeout=0.05)

    if args.mode != "static" and not args.skip_warmup:
        run(ser, lambda _t: (0.0, 0.0, -GRAVITY), lambda _t: (0.0, 0.0, 0.0),
            args.warmup, "calibration warm-up: level, stationary")

    if args.mode == "static":
        run(ser, lambda _t: (0.0, 0.0, -GRAVITY), lambda _t: (0.0, 0.0, 0.0),
            args.duration, "static level, stationary")

    elif args.mode == "tilt":
        r = math.radians(args.roll_deg)
        # Gravity's specific-force reading rotated into the body frame by a
        # roll of `r` about body X. Level: az=-G (see imu_cal.rs comment).
        acc = (0.0, GRAVITY * math.sin(r), -GRAVITY * math.cos(r))
        run(ser, lambda _t: acc, lambda _t: (0.0, 0.0, 0.0),
            args.duration, f"static {args.roll_deg:.0f} deg roll, stationary")

    elif args.mode == "dynamic":
        run(ser, lambda _t: (0.0, 0.0, -GRAVITY), lambda _t: (args.roll_rate, 0.0, 0.0),
            args.duration, f"constant roll rate gyr_x={args.roll_rate} rad/s")

    ser.close()


if __name__ == "__main__":
    main()
