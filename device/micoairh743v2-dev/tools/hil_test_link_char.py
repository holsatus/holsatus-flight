# phase0_link_char.py
import serial, struct, sys, time, statistics
port, baud = sys.argv[1], int(sys.argv[2])
FRAME = 32
ser = serial.Serial(port, baud, timeout=0.05)
rtts = []
for seq in range(50000):
    pkt = struct.pack("<BBI", 0xA5, 0x01, seq) + b"\x00" * (FRAME - 6)
    t0 = time.perf_counter()
    ser.write(pkt)
    echo = ser.read(FRAME)
    t1 = time.perf_counter()
    if len(echo) == FRAME:
        rtts.append((t1 - t0) * 1e6)   # microseconds
    if seq % 1000 == 0:
        print(f"seq={seq} rtt={(t1-t0)*1e6:.0f}us")
rtts.sort()
p = lambda q: rtts[int(q * len(rtts))]
print(f"n={len(rtts)} median={statistics.median(rtts):.1f}us "
      f"p99={p(0.99):.1f}us max={rtts[-1]:.1f}us "
      f"jitter(sd)={statistics.pstdev(rtts):.1f}us")
