"""
Viser rå 24-bit samples og statistik for at diagnosticere støjkilden.
"""
import serial
import serial.tools.list_ports
import time, sys, math

PORT  = None
BAUD  = 115200
N     = 500   # antal samples at analysere

def find_port():
    for p in serial.tools.list_ports.comports():
        if "teensy" in (p.description or "").lower() or "usb serial" in (p.description or "").lower():
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None

port = PORT or find_port()
ser  = serial.Serial(port, BAUD, timeout=2)
time.sleep(0.5)
ser.reset_input_buffer()

print(f"Indsamler {N} samples...\n")

L_vals, R_vals = [], []
while len(L_vals) < N:
    raw = ser.readline().decode("utf-8", errors="ignore").strip()
    if "," not in raw:
        continue
    try:
        l, r = raw.split(",")
        L_vals.append(int(l))
        R_vals.append(int(r))
    except:
        continue

ser.close()

# Print første 10 samples råt
print("=== Første 10 samples (rå 24-bit signed) ===")
for i in range(10):
    print(f"  [{i:3d}]  L: {L_vals[i]:+10d}   R: {R_vals[i]:+10d}")

print()

# Statistik
for name, vals in [("L", L_vals), ("R", R_vals)]:
    mean  = sum(vals) / len(vals)
    rms   = math.sqrt(sum(v*v for v in vals) / len(vals))
    mn    = min(vals)
    mx    = max(vals)
    peak  = max(abs(mn), abs(mx))
    dbfs  = 20 * math.log10(rms / 8388608) if rms > 0 else -999
    dc_db = 20 * math.log10(abs(mean) / 8388608) if abs(mean) > 0 else -999

    print(f"=== Kanal {name} ===")
    print(f"  Mean (DC offset): {mean:+.1f} LSB   ({dc_db:.1f} dBFS)")
    print(f"  RMS:              {rms:.1f} LSB   ({dbfs:.2f} dBFS)")
    print(f"  Min/Max:          {mn} / {mx}")
    print(f"  Peak:             {peak} LSB")
    print(f"  Crest factor:     {peak/rms:.2f}")
    print()