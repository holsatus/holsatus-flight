#!/usr/bin/env python3
"""
Phase 1 HIL host: inject a synthetic IMU stream over the USART1 link and
read back the FC's AHRS_ATTITUDE_Q debug frame. Lockstep -- one SensorFrame
in, one AttitudeFrame reply out, per references/hil_implementation_plan.md.

Wire protocol (must match src/bin/flight.rs SENSOR_FRAME_*/ATTITUDE_FRAME_*;
the former hil.rs binary is merged into flight.rs, which enters HIL mode
automatically when the FC boots on USB power):

  SensorFrame (host -> FC, 32 bytes, little-endian):
    u8  sync=0xA5, u8 type=0x01, u32 seq, f32 acc[3], f32 gyr[3], u16 crc16

  AttitudeFrame (FC -> host, 25 bytes, little-endian):
    u8  sync=0x5A, u8 type=0x02, u32 seq, f32 q[4] (i,j,k,w), u8 flags, u16 crc16
    flags bit0 = imu_cal::CAL_DONE. Before it's set, q is hil_link_task's
    identity fallback (att_estimator isn't spawned yet), not a real estimate.

CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over all preceding
bytes in the frame -- matches flight.rs::crc16_ccitt exactly.

Every mode waits for CAL_DONE (streaming level/stationary data) before
switching to its actual profile -- see wait_for_calibration(). Guessing a
fixed warm-up duration doesn't work: the bias-capture window's length
depends on the link's effective sample rate, not the "~2s at 1kHz" real
hardware assumes, so a fixed sleep can straddle the transition and bake a
tilted sample into the bias (see imu_cal.rs, D000077-style).

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
# while the host thought it was running at 2M. See flight.rs HIL_LINK_BAUD.
SENSOR_FRAME_LEN = 32
ATTITUDE_FRAME_LEN = 25
ATTITUDE_FRAME_SYNC = 0x5A
ATTITUDE_FRAME_TYPE = 0x02
ATTITUDE_FLAG_CAL_DONE = 1 << 0
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
    if len(buf) != ATTITUDE_FRAME_LEN or buf[0] != ATTITUDE_FRAME_SYNC or buf[1] != ATTITUDE_FRAME_TYPE:
        return None
    body, crc_rx = buf[:23], struct.unpack("<H", buf[23:25])[0]
    if crc16_ccitt(body) != crc_rx:
        print("CRC-16 check did not pass.")
        return None
    seq, i, j, k, w, flags = struct.unpack("<IffffB", buf[2:23])
    return {
        "seq": seq,
        "quat_ijkw": (i, j, k, w),
        "cal_done": bool(flags & ATTITUDE_FLAG_CAL_DONE),
    }


class AttitudeFrameReader:
    """Buffered, resyncing reader for AttitudeFrame replies.

    The FC's reply path shares the executor with SD/FAT mount, loggers,
    motor governor etc. at boot, so a reply can arrive late or truncated
    even though the matched-baud Phase 0 echo test (no such contention)
    measured a clean ~6ms RTT. A fixed-size ser.read(ATTITUDE_FRAME_LEN)
    treats byte 0 of that read as frame start unconditionally: one
    truncated read leaves stray bytes in the OS buffer and desyncs every
    later frame for good. This instead accumulates bytes across calls and
    scans for the sync/type pair, dropping one byte at a time on a CRC
    miss, so a single bad read can only cost one frame, not the rest of
    the run.
    """

    def __init__(self):
        self.buf = bytearray()

    def read(self, ser, timeout_s: float):
        deadline = time.perf_counter() + timeout_s
        while True:
            frame = self._try_extract()
            if frame is not None:
                return frame
            remaining = deadline - time.perf_counter()
            if remaining <= 0:
                return None
            # pyserial's read(size) blocks until `size` bytes arrive or its
            # own timeout expires -- it does NOT return early just because
            # some bytes showed up. Lockstep means nothing more than one
            # frame is ever in flight, so requesting more than what's left
            # to complete a candidate frame would burn the full per-call
            # timeout waiting for bytes that aren't coming (measured: this
            # turned a ~6ms round trip into a ~54ms one).
            need = max(1, ATTITUDE_FRAME_LEN - len(self.buf))
            chunk = ser.read(need)
            if not chunk and time.perf_counter() >= deadline:
                return None
            self.buf.extend(chunk)

    def _try_extract(self):
        while True:
            idx = self.buf.find(bytes([ATTITUDE_FRAME_SYNC]))
            if idx == -1:
                self.buf.clear()
                return None
            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < 2:
                return None
            if self.buf[1] != ATTITUDE_FRAME_TYPE:
                del self.buf[0:1]
                continue
            if len(self.buf) < ATTITUDE_FRAME_LEN:
                return None
            candidate = bytes(self.buf[:ATTITUDE_FRAME_LEN])
            att = parse_attitude_frame(candidate)
            if att is None:
                # Sync/type matched but CRC didn't -- a false sync inside
                # unrelated bytes, not a real frame. Drop just the sync
                # byte and keep scanning instead of trusting it.
                del self.buf[0:1]
                continue
            del self.buf[:ATTITUDE_FRAME_LEN]
            return att


def quat_to_euler_deg(i, j, k, w):
    # Aerospace roll-pitch-yaw (ZYX intrinsic) extraction. Sign/axis
    # convention should match nalgebra's UnitQuaternion::euler_angles() --
    # sanity-check against a known static test before trusting magnitudes.
    roll = math.atan2(2 * (w * i + j * k), 1 - 2 * (i * i + j * j))
    sinp = max(-1.0, min(1.0, 2 * (w * j - k * i)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * k + i * j), 1 - 2 * (j * j + k * k))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def wait_for_calibration(ser, max_wait_s: float) -> bool:
    """Stream level/stationary data until the firmware reports CAL_DONE.

    Returns True once confirmed, False if max_wait_s elapses first (in
    which case the caller's actual test would just be reading the identity
    fallback, not a real estimate -- worth a loud warning, not a silent
    proceed).
    """
    print(f"-- calibration warm-up: level, stationary (up to {max_wait_s:.0f}s) --")
    t0 = time.perf_counter()
    seq = 0
    reader = AttitudeFrameReader()
    while True:
        t = time.perf_counter() - t0
        if t > max_wait_s:
            print(f"WARNING: CAL_DONE not seen after {max_wait_s:.0f}s -- proceeding anyway; "
                  f"replies will likely be the identity fallback, not a real estimate\n")
            return False
        ser.write(build_sensor_frame(seq, (0.0, 0.0, -GRAVITY), (0.0, 0.0, 0.0)))
        att = reader.read(ser, ser.timeout)
        if att is not None and att["cal_done"]:
            print(f"calibration confirmed done at t={t:.2f}s (seq={seq})\n")
            return True
        seq += 1


def run(ser, acc_fn, gyr_fn, duration_s: float, label: str):
    print(f"-- {label} ({duration_s:.1f}s) --")
    t0 = time.perf_counter()
    seq = 0
    last_print = 0.0
    n_bad = 0
    reader = AttitudeFrameReader()
    while time.perf_counter() - t0 < duration_s:
        t = time.perf_counter() - t0
        ser.write(build_sensor_frame(seq, acc_fn(t), gyr_fn(t)))
        att = reader.read(ser, ser.timeout)
        if att is None:
            n_bad += 1
        elif t - last_print > 0.2:
            last_print = t
            r, p, y = quat_to_euler_deg(*att["quat_ijkw"])
            cal = "Y" if att["cal_done"] else "N"
            print(f"t={t:5.2f}s seq={seq:6d} cal={cal} roll={r:6.2f} pitch={p:6.2f} yaw={y:6.2f}")
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
    ap.add_argument("--warmup-timeout", type=float, default=30.0,
                     help="max seconds to wait for imu_cal::CAL_DONE before giving up and "
                          "running the requested mode anyway (see wait_for_calibration)")
    ap.add_argument("--skip-warmup", action="store_true",
                     help="skip waiting for CAL_DONE (only safe if calibration already "
                          "completed in the current boot session, e.g. right after a prior "
                          "run against the same, still-powered board)")
    args = ap.parse_args()

    import serial
    ser = serial.Serial(args.port, args.baud, timeout=0.05)

    if args.mode != "static" and not args.skip_warmup:
        wait_for_calibration(ser, args.warmup_timeout)

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
