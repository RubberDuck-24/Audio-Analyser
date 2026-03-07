"""
Audio Analyzer — Serial Multimeter
Læser L/R samples fra Teensy, beregner RMS og viser det som et multimeter.

Krav: pip install pyserial
Brug: python multimeter.py (justér PORT nedenfor)
"""

import serial
import serial.tools.list_ports
import math
import time
import sys

# ── Indstillinger ──────────────────────────────────────────
PORT        = None      # None = auto-find Teensy
BAUD        = 115200
SAMPLES_N   = 50        # Antal samples per måling (RMS-vindue)
MAX_VAL     = 32768     # 16-bit signed max (bruges til dBFS beregning)
BAR_WIDTH   = 30        # Bredde på bar-indikatoren
# ───────────────────────────────────────────────────────────


def find_teensy_port():
    """Prøver at finde Teensy automatisk på tilgængelige COM-porte."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        mfg  = (p.manufacturer or "").lower()
        if "teensy" in desc or "teensy" in mfg or "usb serial" in desc:
            return p.device
    # Fallback: returner første tilgængelige port
    if ports:
        return ports[0].device
    return None


def rms(values):
    """Beregner RMS af en liste af tal."""
    if not values:
        return 0.0
    return math.sqrt(sum(v * v for v in values) / len(values))


def to_dbfs(rms_val):
    """Konverterer RMS til dBFS (0 dBFS = fuld skala)."""
    if rms_val <= 0:
        return -999.0
    return 20 * math.log10(rms_val / MAX_VAL)


def bar(rms_val, width=BAR_WIDTH):
    """Tegner en simpel ASCII bar baseret på dBFS niveau."""
    db = to_dbfs(rms_val)
    # Skala: -80 dBFS til 0 dBFS
    pct = max(0.0, min(1.0, (db + 80) / 80))
    filled = int(pct * width)
    b = "█" * filled + "░" * (width - filled)
    return b


def main():
    # Find port
    port = PORT or find_teensy_port()
    if not port:
        print("FEJL: Ingen serial port fundet.")
        print("Tilslut Teensy og prøv igen, eller sæt PORT manuelt i scriptet.")
        sys.exit(1)

    print(f"Forbinder til {port} @ {BAUD} baud...")
    try:
        ser = serial.Serial(port, BAUD, timeout=2)
    except serial.SerialException as e:
        print(f"FEJL: Kunne ikke åbne port: {e}")
        sys.exit(1)

    time.sleep(0.5)   # Giv Teensy tid til at starte
    ser.reset_input_buffer()

    print(f"Forbundet! Måler med vindue på {SAMPLES_N} samples.")
    print("Tryk Ctrl+C for at stoppe.\n")

    left_buf  = []
    right_buf = []
    errors    = 0

    try:
        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line or "," not in line:
                continue

            parts = line.split(",")
            if len(parts) != 2:
                continue

            try:
                l_val = int(parts[0])
                r_val = int(parts[1])
            except ValueError:
                errors += 1
                continue

            left_buf.append(l_val)
            right_buf.append(r_val)

            # Når vi har nok samples, beregn og vis
            if len(left_buf) >= SAMPLES_N:

                rms_l = rms(left_buf)
                rms_r = rms(right_buf)
                db_l  = to_dbfs(rms_l)
                db_r  = to_dbfs(rms_r)

                # Byg display-linje (overskriv forrige linje med \r)
                line_l = f"L: {bar(rms_l)} {db_l:7.2f} dBFS  ({rms_l:6.1f} LSB)"
                line_r = f"R: {bar(rms_r)} {db_r:7.2f} dBFS  ({rms_r:6.1f} LSB)"

                # Flyt cursor op 2 linjer og overskriv
                sys.stdout.write("\033[2A")   # op 2 linjer
                sys.stdout.write(f"{line_l}\n")
                sys.stdout.write(f"{line_r}\n")
                sys.stdout.flush()

                # Ryd buffere
                left_buf.clear()
                right_buf.clear()

    except KeyboardInterrupt:
        print("\n\nStoppet.")
        ser.close()


if __name__ == "__main__":
    # Print to tomme linjer som placeholder for display
    print("")
    print("")
    main()