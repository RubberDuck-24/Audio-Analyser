"""
Audio Analyzer — Serial Multimeter  (v2, 24-bit)
Læser L/R samples fra Teensy, beregner RMS og viser som multimeter.

Krav: pip install pyserial
Brug: python multimeter.py
"""

import serial
import serial.tools.list_ports
import math
import sys
import time

# ── Indstillinger ─────────────────────────────────────────────────────────
PORT       = None     # None = auto-find Teensy
BAUD       = 115200
SAMPLES_N  = 200      # RMS vindue i samples
MAX_VAL    = 8388608  # 24-bit signed maks (2^23)
BAR_WIDTH  = 30
# ──────────────────────────────────────────────────────────────────────────


def find_teensy():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfg  = (p.manufacturer or "").lower()
        if "teensy" in desc or "teensy" in mfg or "usb serial" in desc:
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


def rms(values):
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def to_dbfs(rms_val):
    if rms_val <= 0:
        return -999.0
    return 20.0 * math.log10(rms_val / MAX_VAL)


def bar(rms_val, width=BAR_WIDTH):
    db  = to_dbfs(rms_val)
    pct = max(0.0, min(1.0, (db + 120) / 120))   # skala: -120 til 0 dBFS
    n   = int(pct * width)
    # Farve: grøn under -20, gul -20 til -6, rød over -6
    if db > -6:
        color = "\033[91m"   # rød
    elif db > -20:
        color = "\033[93m"   # gul
    else:
        color = "\033[92m"   # grøn
    reset = "\033[0m"
    return f"{color}{'█' * n}{'░' * (width - n)}{reset}"


def main():
    port = PORT or find_teensy()
    if not port:
        print("FEJL: Ingen port fundet. Sæt PORT manuelt i scriptet.")
        sys.exit(1)

    print(f"Forbinder til {port} @ {BAUD} baud...")
    try:
        ser = serial.Serial(port, BAUD, timeout=2)
    except serial.SerialException as e:
        print(f"FEJL: {e}")
        sys.exit(1)

    time.sleep(0.5)
    ser.reset_input_buffer()
    print(f"Forbundet!  Vindue: {SAMPLES_N} samples  |  Maks: ±{MAX_VAL} (24-bit)")
    print("Tryk Ctrl+C for at stoppe.\n")

    L_buf, R_buf = [], []
    errors = 0

    try:
        while True:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw or "," not in raw:
                continue
            parts = raw.split(",")
            if len(parts) != 2:
                continue
            try:
                L_buf.append(int(parts[0]))
                R_buf.append(int(parts[1]))
            except ValueError:
                errors += 1
                continue

            if len(L_buf) >= SAMPLES_N:
                rms_l = rms(L_buf)
                rms_r = rms(R_buf)
                db_l  = to_dbfs(rms_l)
                db_r  = to_dbfs(rms_r)

                line_l = f"L: {bar(rms_l)} {db_l:8.2f} dBFS  ({rms_l:9.1f} / {MAX_VAL})"
                line_r = f"R: {bar(rms_r)} {db_r:8.2f} dBFS  ({rms_r:9.1f} / {MAX_VAL})"

                sys.stdout.write("\033[2A")
                sys.stdout.write(f"{line_l}\n")
                sys.stdout.write(f"{line_r}\n")
                sys.stdout.flush()

                L_buf.clear()
                R_buf.clear()

    except KeyboardInterrupt:
        print("\n\nStoppet.")
        ser.close()


if __name__ == "__main__":
    print("")
    print("")
    main()