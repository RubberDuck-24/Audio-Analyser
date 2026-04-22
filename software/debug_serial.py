"""
debug_serial.py — Teensy PCM4222 pakke-diagnostik
===================================================
Viser:
  - Pakkemodtagelse og sekvensgab (= tabte pakker)
  - Faktiske sample-værdier (L og R kanal)
  - Estimeret frekvens på R-kanal (til at verificere ADC)
  - Båndbredde og pakke-rate
"""
import serial
import serial.tools.list_ports
import struct
import time
import numpy as np

BAUD        = 2_000_000
MAGIC       = 0x30445541
HEADER      = struct.Struct("<III")
MAGIC_BYTES = struct.pack("<I", MAGIC)
FS          = 48_000        # sæt til 192000 hvis ingen decimation i firmware

TEST_SECS   = 8


def find_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "teensy" in desc or "pjrc" in (p.manufacturer or "").lower():
            return p.device
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None


def estimate_freq(samples: np.ndarray, fs: float):
    if len(samples) < 64:
        return None
    y = samples - np.mean(samples)
    if np.max(np.abs(y)) < 10:
        return None
    zc = np.where((y[:-1] < 0) & (y[1:] >= 0))[0]
    if len(zc) < 2:
        return None
    periods = np.diff(zc) / fs
    return 1.0 / np.median(periods)


port = find_port()
if not port:
    print("FEJL: Ingen serial port fundet!")
    exit(1)

print(f"Port: {port}  Baud: {BAUD}")
print(f"Forventet FS: {FS} Hz")
print("-" * 60)

ser = serial.Serial(
    port, BAUD,
    timeout=0.1,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)

# Drain Windows USB CDC buffer — Teensy har sendt data siden boot
# Læs og smid væk i 1.5 sekunder så vi starter med friske pakker
print("Drainer startup-buffer (1.5 sek)...")
drain_end = time.monotonic() + 1.5
while time.monotonic() < drain_end:
    try:
        waiting = ser.in_waiting
        if waiting:
            ser.read(waiting)
    except Exception:
        pass
    time.sleep(0.05)
ser.reset_input_buffer()
print("Klar.\n")

rx           = bytearray()
packets      = 0
drops        = 0
last_seq     = None
total_words  = 0
all_left     = []
all_right    = []
start        = time.monotonic()
last_report  = start

print(f"Leder efter pakker i {TEST_SECS} sekunder...\n")

while time.monotonic() - start < TEST_SECS:
    try:
        waiting = ser.in_waiting
    except (serial.SerialException, OSError):
        waiting = 0

    try:
        chunk = ser.read(max(1, waiting))
    except (serial.SerialException, OSError) as e:
        print(f"Serial fejl: {e}")
        break

    if not chunk:
        continue

    rx.extend(chunk)

    while True:
        idx = rx.find(MAGIC_BYTES)
        if idx < 0:
            if len(rx) > 3:
                del rx[:-3]
            break
        if idx > 0:
            del rx[:idx]
        if len(rx) < HEADER.size:
            break

        magic, words, seq = HEADER.unpack_from(rx, 0)
        if magic != MAGIC or words == 0 or words > 65536 or words % 2:
            del rx[0]
            continue

        pkt_size = HEADER.size + words * 4
        if len(rx) < pkt_size:
            break

        payload = bytes(rx[HEADER.size:pkt_size])
        del rx[:pkt_size]

        # Sekvensgab
        if last_seq is not None:
            gap = (seq - last_seq - 1) & 0xFFFF_FFFF
            if gap > 0:
                drops += gap
                print(f"  ⚠ GAB: seq {last_seq} → {seq}  ({gap} pakker tabt)")
        last_seq = seq

        raw   = np.frombuffer(payload, dtype="<i4") >> 8
        left  = raw[0::2]
        right = raw[1::2]

        all_left.extend(left.tolist())
        all_right.extend(right.tolist())

        packets    += 1
        total_words += words

        # Print første 3 pakker detaljeret
        if packets <= 3:
            print(f"Pakke #{seq}:  words={words}  bytes={pkt_size}")
            print(f"  L: min={left.min():+9d}  max={left.max():+9d}"
                  f"  rms={int(np.sqrt(np.mean(left.astype(np.float64)**2))):7d}"
                  f"  mean={int(np.mean(left)):+9d}")
            print(f"  R: min={right.min():+9d}  max={right.max():+9d}"
                  f"  rms={int(np.sqrt(np.mean(right.astype(np.float64)**2))):7d}"
                  f"  mean={int(np.mean(right)):+9d}")
            # Første 8 raw sample-par
            print(f"  Første 8 sample-par (L, R):")
            for i in range(min(8, len(left))):
                print(f"    [{i}]  L={left[i]:+9d}   R={right[i]:+9d}")
            print()

        # Live rapport hvert sekund
        now = time.monotonic()
        if now - last_report >= 1.0:
            elapsed  = now - start
            pkt_rate = packets / elapsed
            bw_kbps  = (total_words * 4 * 8) / elapsed / 1000
            print(f"[{elapsed:4.1f}s]  pakker={packets:5d}  drops={drops:4d}"
                  f"  rate={pkt_rate:5.1f} pkt/s  båndbredde={bw_kbps:.0f} kbit/s")
            last_report = now

ser.close()

# ── Samlet analyse ────────────────────────────────────────────────────────
elapsed  = time.monotonic() - start
print()
print("=" * 60)
print("SAMLET ANALYSE")
print("=" * 60)
print(f"  Pakker modtaget : {packets}")
print(f"  Pakker tabt     : {drops}  ({100*drops/max(1,packets+drops):.1f}%)")
print(f"  Total samples   : {len(all_left)} per kanal")
print(f"  Varighed        : {elapsed:.1f} s")
print(f"  Pakke-rate      : {packets/elapsed:.1f} pkt/s  "
      f"(forventet ≈ {FS//(512//2):.0f} ved FS={FS})")

if len(all_left) > 256:
    al = np.array(all_left,  dtype=np.float64)
    ar = np.array(all_right, dtype=np.float64)

    print()
    print("KANAL L:")
    print(f"  DC-offset : {np.mean(al):+.0f} counts")
    print(f"  RMS       : {np.sqrt(np.mean(al**2)):.0f} counts")
    print(f"  Vpp       : {np.ptp(al):.0f} counts")
    freq_l = estimate_freq(al[-4096:], FS)
    print(f"  Frekvens  : {freq_l:.1f} Hz" if freq_l else "  Frekvens  : ikke detekteret")

    print()
    print("KANAL R:")
    print(f"  DC-offset : {np.mean(ar):+.0f} counts")
    print(f"  RMS       : {np.sqrt(np.mean(ar**2)):.0f} counts")
    print(f"  Vpp       : {np.ptp(ar):.0f} counts")
    freq_r = estimate_freq(ar[-4096:], FS)
    print(f"  Frekvens  : {freq_r:.1f} Hz" if freq_r else "  Frekvens  : ikke detekteret")

    print()
    # Tjek om L og R er identiske (tegn på forkert L/R mapping)
    if np.allclose(al[:256], ar[:256], atol=1):
        print("⚠  L og R er identiske — mulig L/R swap fejl i firmware")
    # Tjek om data er rent nul
    if np.max(np.abs(al)) < 10 and np.max(np.abs(ar)) < 10:
        print("⚠  BEGGE kanaler er nær nul — ADC modtager ingen signal?")
        print("   Tjek: PCM4222 tændt? BCLK/LRCK fra EVM til Teensy?")
    elif np.max(np.abs(al)) < 10:
        print("⚠  Kanal L er nær nul — kun R har signal")
    elif np.max(np.abs(ar)) < 10:
        print("⚠  Kanal R er nær nul — kun L har signal")
    else:
        print("✓  Begge kanaler har signal")

print()
if drops == 0:
    print("✓  Ingen pakketab — serial stream er stabil")
elif drops / max(1, packets + drops) < 0.01:
    print(f"✓  Under 1% tab — acceptabelt")
else:
    print(f"⚠  {drops} pakker tabt ({100*drops/max(1,packets+drops):.1f}%)")
    print("   Prøv: lavere BLOCK_PAIRS i firmware, eller luk andre programmer")