"""
Audio Analyzer — FFT Spektrum, FSL-format
Teensy 4.1 + PCM4222EVM, 24-bit binær protokol

Krav: pip install pyserial numpy matplotlib
"""

import struct
import sys
import time
import threading
import collections

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import serial.tools.list_ports
from matplotlib.ticker import FuncFormatter

def make_lowpass_sos(cutoff_hz, fs, order=4):
    """Butterworth LP filter via bilinear transform — ingen scipy nødvendig."""
    from numpy import pi, tan, zeros, ones, array
    wc = tan(pi * cutoff_hz / fs)
    wc2 = wc * wc
    sos = []
    for k in range(order // 2):
        theta = pi * (2*k + 1) / (2 * order)
        b0 = wc2 / (1 + 2*wc*np.sin(theta) + wc2)
        b1 = 2 * b0
        b2 = b0
        a1 = 2 * (wc2 - 1) / (1 + 2*wc*np.sin(theta) + wc2)
        a2 = (1 - 2*wc*np.sin(theta) + wc2) / (1 + 2*wc*np.sin(theta) + wc2)
        sos.append([b0, b1, b2, 1.0, a1, a2])
    return np.array(sos)

def sosfilt_np(sos, x):
    """LP filter deaktiveret — returnerer signal uændret."""
    return x

LP_CUTOFF_HZ = 40_000  # LP filter cutoff — juster frit

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                      KONFIGURATION                                      ║
# ╠══════════════════════════════════════════════════════════════════════════╣

# ── Instrument-indstillinger (som på FSL) ─────────────────────────────────
CENTER_FREQ = 10_000     # Hz  — center frekvens
SPAN        = 50_000     # Hz  — total span (viser CENTER ± SPAN/2)
REF_LEVEL   = 10         # dBm — øverste Y-linje
DB_RANGE    = 100        # dB  — Y-aksens højde

# ── Kalibrering ───────────────────────────────────────────────────────────
CAL_SIGNAL_VPP  = 2.12   # Faktisk målt Vpp ved 1Vpp indstilling (generator High Z + 50Ω term)
CAL_Z           = 50.0   # Ohm — impedans (FSL = 50Ω)
CAL_CHANNEL     = "R"    # "L" eller "R" — kanal med kalibreringssignal
CAL_FRAMES      = 30     # Antal frames til at estimere offset

# ── FFT ───────────────────────────────────────────────────────────────────
FS              = 192_000  # Sample rate Hz
FFT_SIZE        = 16_384   # 16384 → 11.7 Hz/bin, hurtigere end 65536
AVG_FRAMES      = 20
PEAK_DECAY      = 0.5      # dB/frame

# ── Harmoniske ────────────────────────────────────────────────────────────
SHOW_HARMONICS  = True
N_HARMONICS     = 5

# ── Serial ────────────────────────────────────────────────────────────────
PORT            = None
BAUD            = 2_000_000
BUFFER_SECS     = 2

# ╚══════════════════════════════════════════════════════════════════════════╝

FREQ_MIN  = max(CENTER_FREQ - SPAN / 2, 0)
FREQ_MAX  = CENTER_FREQ + SPAN / 2
DB_MIN    = REF_LEVEL - DB_RANGE
DB_MAX    = REF_LEVEL

CAL_VRMS  = (CAL_SIGNAL_VPP / 2) / np.sqrt(2)
CAL_DBV   = 20 * np.log10(CAL_VRMS)
CAL_DBM   = CAL_DBV - 10 * np.log10(CAL_Z / 1000)

BUFFER_LEN  = int(FS * BUFFER_SECS)
LP_SOS      = make_lowpass_sos(LP_CUTOFF_HZ, FS)
HEADER      = struct.Struct("<III")
MAGIC       = 0x30445541
MAGIC_BYTES = struct.pack("<I", MAGIC)
MAX_WORDS   = 65536

L_buf = collections.deque(maxlen=BUFFER_LEN)
R_buf = collections.deque(maxlen=BUFFER_LEN)
lock  = threading.Lock()
stop_event = threading.Event()

packet_drops = 0
last_seq     = None
dbfs_to_dbm_offset = None
cal_samples = []


def find_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower() if p.manufacturer else ""
        if "teensy" in desc or "pjrc" in manu:
            return p.device
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if len(ports) == 1 else None


def serial_reader(port):
    global packet_drops, last_seq
    try:
        ser = serial.Serial(port, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"FEJL: {e}")
        return
    time.sleep(0.3)
    ser.reset_input_buffer()
    print(f"Forbundet: {port}")
    rx = bytearray()

    while not stop_event.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 4096)
        except serial.SerialException:
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
            if magic != MAGIC or words == 0 or words > MAX_WORDS or words % 2:
                del rx[0]
                continue

            pkt_size = HEADER.size + words * 4
            if len(rx) < pkt_size:
                break

            payload = bytes(rx[HEADER.size:pkt_size])
            del rx[:pkt_size]

            if last_seq is not None and seq != (last_seq + 1) & 0xFFFFFFFF:
                packet_drops += (seq - last_seq - 1) & 0xFFFFFFFF
            last_seq = seq

            data = np.frombuffer(payload, dtype="<i4") >> 8
            with lock:
                L_buf.extend(data[0::2])
                R_buf.extend(data[1::2])

    try:
        ser.close()
    except Exception:
        pass


def compute_fft(samples):
    n = len(samples)
    x = samples.astype(np.float64) - np.mean(samples)
    x = sosfilt_np(LP_SOS, x)                   # LP-filter inden FFT
    w = np.hanning(n)
    cg = np.mean(w)
    spec = np.fft.rfft(x * w)
    mag = np.abs(spec) / (n * cg)
    if len(mag) > 2:
        mag[1:-1] *= 2.0
    mag_db = 20 * np.log10(np.maximum(mag / (2**23), 1e-12))
    freqs = np.fft.rfftfreq(n, d=1.0 / FS)
    mask = (freqs >= FREQ_MIN) & (freqs <= FREQ_MAX)
    return freqs[mask], mag_db[mask]


# ── Plot setup ────────────────────────────────────────────────────────────
fig, (ax_l, ax_r) = plt.subplots(2, 1, figsize=(13, 8), sharex=True,
                                   gridspec_kw={"hspace": 0.12})
fig.patch.set_facecolor("#1a1a2e")
fig.suptitle("Audio Analyzer — FFT Spektrum (24-bit, 192 kHz)",
             color="white", fontsize=13, fontweight="bold")

_f0 = np.linspace(FREQ_MIN, FREQ_MAX, 512)
_d0 = np.full(512, DB_MIN)

for ax, title, col in [(ax_l, "Kanal L", "#00d4ff"),
                        (ax_r, "Kanal R", "#ff6b6b")]:
    ax.set_facecolor("#0d0d1a")
    ax.set_xlim(FREQ_MIN, FREQ_MAX)
    ax.set_ylim(DB_MIN, DB_MAX)
    ax.set_ylabel("dBm  (50 Ω)", color="white", fontsize=10)
    ax.set_title(title, color=col, fontsize=11, loc="left", pad=4)
    ax.tick_params(colors="white", labelsize=9)
    for sp in ax.spines.values():
        sp.set_color("#444466")
    ax.grid(True, color="#2a2a4a", linewidth=0.7, linestyle="--")
    ax.set_yticks(np.arange(DB_MIN, DB_MAX + 1, 10))
    # Vertikale referencemarkeringer
    step = SPAN / 5
    for f in np.arange(FREQ_MIN, FREQ_MAX + 1, step):
        ax.axvline(f, color="#333355", linewidth=0.8, linestyle=":")
        label = f"{f/1000:.0f}k" if f >= 1000 else f"{int(f)}"
        ax.text(f, DB_MAX - 2, label, color="#666688",
                fontsize=7, ha="center", va="top")

for ax in (ax_l, ax_r):
    ax.xaxis.set_major_formatter(
        FuncFormatter(lambda x, _: f"{x/1000:.0f}k" if x >= 1000 else f"{int(x)}"))

ax_l.tick_params(labelbottom=False)
ax_r.set_xlabel("Frekvens (Hz)", color="white", fontsize=10)

line_l, = ax_l.plot(_f0, _d0, color="#00d4ff", linewidth=0.9)
fill_l  = ax_l.fill_between(_f0, DB_MIN, _d0, color="#00d4ff", alpha=0.15)
peak_line_l, = ax_l.plot(_f0, _d0, color="#80eeff", linewidth=0.7,
                          linestyle="--", alpha=0.5)
line_r, = ax_r.plot(_f0, _d0, color="#ff6b6b", linewidth=0.9)
fill_r  = ax_r.fill_between(_f0, DB_MIN, _d0, color="#ff6b6b", alpha=0.15)
peak_line_r, = ax_r.plot(_f0, _d0, color="#ffaaaa", linewidth=0.7,
                          linestyle="--", alpha=0.5)

txt_l = ax_l.text(0.99, 0.97, "", transform=ax_l.transAxes, color="#00d4ff",
                   fontsize=9, ha="right", va="top",
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="#0d0d1a",
                             edgecolor="#00d4ff", alpha=0.85))
txt_r = ax_r.text(0.99, 0.97, "", transform=ax_r.transAxes, color="#ff6b6b",
                   fontsize=9, ha="right", va="top",
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="#0d0d1a",
                             edgecolor="#ff6b6b", alpha=0.85))
status_txt = fig.text(0.01, 0.01, "Status: venter...", color="white", fontsize=9)

def make_harm(ax):
    return [(ax.axvline(FREQ_MIN, color="#ffff44", lw=1.0, ls="--", alpha=0.0),
             ax.text(FREQ_MIN, DB_MIN + 8, "", color="#ffff44", fontsize=8,
                     ha="center", va="bottom", fontweight="bold", alpha=0.0,
                     bbox=dict(boxstyle="round,pad=0.2", facecolor="#0d0d1a",
                               edgecolor="#ffff44", alpha=0.0)))
            for _ in range(N_HARMONICS)]

harm_l = make_harm(ax_l)
harm_r = make_harm(ax_r)

plt.tight_layout(rect=[0, 0.03, 1, 0.95])

avg_l = collections.deque(maxlen=AVG_FRAMES)
avg_r = collections.deque(maxlen=AVG_FRAMES)
peak_l_hold = None
peak_r_hold = None


def draw_harmonics(harm, fund_f, fund_dbm):
    active = fund_dbm > (DB_MIN + 40)
    for i, (vline, label) in enumerate(harm):
        hf = fund_f * (i + 2)
        if active and FREQ_MIN < hf < FREQ_MAX:
            vline.set_xdata([hf, hf])
            vline.set_alpha(0.7)
            label.set_x(hf)
            label.set_text(f"H{i+2}  {hf/1000:.1f}k" if hf >= 1000 else f"H{i+2}  {hf:.0f}")
            label.set_alpha(1.0)
            label.get_bbox_patch().set_alpha(0.85)
        else:
            vline.set_alpha(0.0)
            label.set_alpha(0.0)
            label.get_bbox_patch().set_alpha(0.0)


def update(_):
    global fill_l, fill_r, peak_l_hold, peak_r_hold
    global dbfs_to_dbm_offset, cal_samples

    with lock:
        if len(L_buf) < FFT_SIZE or len(R_buf) < FFT_SIZE:
            status_txt.set_text(f"Status: venter... ({len(L_buf)}/{FFT_SIZE} samples)")
            return
        L_arr = np.array(list(L_buf)[-FFT_SIZE:], dtype=np.float64)
        R_arr = np.array(list(R_buf)[-FFT_SIZE:], dtype=np.float64)

    freqs_l, mag_l = compute_fft(L_arr)
    freqs_r, mag_r = compute_fft(R_arr)

    # Averaging i lineært domæne
    avg_l.append(10 ** (mag_l / 20))
    avg_r.append(10 ** (mag_r / 20))
    mag_l = 20 * np.log10(np.mean(avg_l, axis=0))
    mag_r = 20 * np.log10(np.mean(avg_r, axis=0))

    # Kalibrering
    if dbfs_to_dbm_offset is None:
        cal_arr = L_arr if CAL_CHANNEL == "L" else R_arr
        ac = cal_arr - np.mean(cal_arr)
        rms_dbfs = 20 * np.log10(max(np.sqrt(np.mean(ac**2)) / (2**23), 1e-12))
        cal_samples.append(rms_dbfs)
        n = len(cal_samples)
        status_txt.set_text(f"Status: kalibrerer kanal {CAL_CHANNEL}... ({n}/{CAL_FRAMES})")
        if n >= CAL_FRAMES:
            dbfs_to_dbm_offset = CAL_DBM - float(np.mean(cal_samples))
            print(f"Kalibreret! Offset = {dbfs_to_dbm_offset:+.2f} dB  "
                  f"(målt {np.mean(cal_samples):.1f} dBFS → {CAL_DBM:.2f} dBm)")
        return

    # dBFS → dBm
    mag_l_dbm = mag_l + dbfs_to_dbm_offset
    mag_r_dbm = mag_r + dbfs_to_dbm_offset

    # Peak-hold
    if peak_l_hold is None or len(peak_l_hold) != len(mag_l_dbm):
        peak_l_hold = mag_l_dbm.copy()
        peak_r_hold = mag_r_dbm.copy()
    else:
        peak_l_hold = np.maximum(peak_l_hold - PEAK_DECAY, mag_l_dbm)
        peak_r_hold = np.maximum(peak_r_hold - PEAK_DECAY, mag_r_dbm)

    line_l.set_data(freqs_l, mag_l_dbm)
    line_r.set_data(freqs_r, mag_r_dbm)
    peak_line_l.set_data(freqs_l, peak_l_hold)
    peak_line_r.set_data(freqs_r, peak_r_hold)

    fill_l.remove()
    fill_r.remove()
    fill_l = ax_l.fill_between(freqs_l, DB_MIN, mag_l_dbm, color="#00d4ff", alpha=0.15)
    fill_r = ax_r.fill_between(freqs_r, DB_MIN, mag_r_dbm, color="#ff6b6b", alpha=0.15)

    ax_l.set_xlim(FREQ_MIN, FREQ_MAX)
    ax_r.set_xlim(FREQ_MIN, FREQ_MAX)
    ax_l.set_ylim(DB_MIN, DB_MAX)
    ax_r.set_ylim(DB_MIN, DB_MAX)

    idx_l = int(np.argmax(mag_l_dbm))
    idx_r = int(np.argmax(mag_r_dbm))
    pf_l, pd_l = freqs_l[idx_l], mag_l_dbm[idx_l]
    pf_r, pd_r = freqs_r[idx_r], mag_r_dbm[idx_r]

    txt_l.set_text(f"Peak: {pf_l:.0f} Hz  {pd_l:.1f} dBm")
    txt_r.set_text(f"Peak: {pf_r:.0f} Hz  {pd_r:.1f} dBm")

    if SHOW_HARMONICS:
        draw_harmonics(harm_l, pf_l, pd_l)
        draw_harmonics(harm_r, pf_r, pd_r)

    status_txt.set_text(
        f"CF={CENTER_FREQ/1000:.0f}kHz  Span={SPAN/1000:.0f}kHz  "
        f"RBW≈{FS/FFT_SIZE:.1f}Hz  |  Drops={packet_drops}  |  "
        f"Ref={CAL_SIGNAL_VPP:.1f}Vpp={CAL_DBM:.2f}dBm  |  Status: live"
    )


def on_close(_):
    stop_event.set()


def main():
    port = PORT or find_port()
    if not port:
        print("FEJL: Ingen Teensy fundet. Sæt PORT manuelt øverst.")
        sys.exit(1)

    print(f"FFT: {FFT_SIZE} punkter  →  {FS/FFT_SIZE:.1f} Hz/bin")
    print(f"Span: {FREQ_MIN/1000:.1f}–{FREQ_MAX/1000:.1f} kHz")
    print(f"Ref: {CAL_SIGNAL_VPP:.1f} Vpp på kanal {CAL_CHANNEL} = {CAL_DBM:.2f} dBm")
    print("Hav signalet tilsluttet nu — kalibrerer første frames...")

    threading.Thread(target=serial_reader, args=(port,), daemon=True).start()
    fig.canvas.mpl_connect("close_event", on_close)
    ani = animation.FuncAnimation(fig, update, interval=100,
                                   blit=False, cache_frame_data=False)
    plt.show()
    _ = ani  # forhindrer garbage collection


if __name__ == "__main__":
    main()