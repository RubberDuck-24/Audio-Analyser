"""
Audio Analyzer — FFT Spektrum
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

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║                      KONFIGURATION                                      ║
# ╠══════════════════════════════════════════════════════════════════════════╣

FREQ_START  =      0     # Hz  — X-akse start
FREQ_STOP   = 20_000     # Hz  — X-akse slut  (max = FS/2 = 96 kHz ved 192 kHz)
REF_LEVEL   =      0     # dBFS — øverste Y-linje
DB_RANGE    =    110     # dB  — Y-aksens højde (Y går fra 0 til -110 dBFS)

# ── FFT ───────────────────────────────────────────────────────────────────
FS          = 192_000    # Sample rate Hz — ingen decimation (DECIM=1 i firmware)
FFT_SIZE    = 16_384     # 16384 → 11.7 Hz/bin ved 192 kHz
AVG_FRAMES  = 20         # Antal frames til midling
PEAK_DECAY  = 0.5        # dB/frame peak-hold henfald

# ── Markør (manuel) ───────────────────────────────────────────────────────
# Klik på et plot for at sætte en markør — klik igen for at flytte den
MARKER_ENABLED = True

# ── Serial ────────────────────────────────────────────────────────────────
PORT        = None        # None = auto-find Teensy
BAUD        = 2_000_000
BUFFER_SECS = 2.0

# ╚══════════════════════════════════════════════════════════════════════════╝

FREQ_MIN   = max(int(FREQ_START), 0)
FREQ_MAX   = min(int(FREQ_STOP),  FS // 2)
DB_MIN     = REF_LEVEL - DB_RANGE
DB_MAX     = REF_LEVEL
FULL_SCALE = float(2**23)    # 24-bit full scale i counts

BUFFER_LEN  = int(FS * BUFFER_SECS)
HEADER      = struct.Struct("<III")
MAGIC       = 0x30445541
MAGIC_BYTES = struct.pack("<I", MAGIC)
MAX_WORDS   = 65536

# ── Delt tilstand ─────────────────────────────────────────────────────────
_ring_l    = np.zeros(BUFFER_LEN, dtype=np.int32)
_ring_r    = np.zeros(BUFFER_LEN, dtype=np.int32)
_write_ptr = 0
_ptr_lock  = threading.Lock()

packet_drops = 0
stop_event   = threading.Event()


# ── Ring-buffer ────────────────────────────────────────────────────────────
def ring_write(left: np.ndarray, right: np.ndarray):
    global _write_ptr
    n   = len(left)
    ptr = _write_ptr % BUFFER_LEN
    end = ptr + n
    if end <= BUFFER_LEN:
        _ring_l[ptr:end] = left
        _ring_r[ptr:end] = right
    else:
        first = BUFFER_LEN - ptr
        _ring_l[ptr:]           = left[:first]
        _ring_l[:n - first]     = left[first:]
        _ring_r[ptr:]           = right[:first]
        _ring_r[:n - first]     = right[first:]
    with _ptr_lock:
        _write_ptr += n


def ring_available() -> int:
    return min(_write_ptr, BUFFER_LEN)


def ring_read_last(n: int):
    with _ptr_lock:
        ptr = _write_ptr
    start = (ptr - n) % BUFFER_LEN
    if start + n <= BUFFER_LEN:
        return _ring_l[start:start + n].copy(), _ring_r[start:start + n].copy()
    cut = BUFFER_LEN - start
    l = np.concatenate([_ring_l[start:], _ring_l[:n - cut]])
    r = np.concatenate([_ring_r[start:], _ring_r[:n - cut]])
    return l, r


# ── Serial reader (thread) ─────────────────────────────────────────────────
def find_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower() if p.manufacturer else ""
        if "teensy" in desc or "pjrc" in manu:
            return p.device
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if len(ports) == 1 else None


def serial_reader(port: str):
    global packet_drops

    try:
        ser = serial.Serial(
            port, BAUD,
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
    except serial.SerialException as e:
        print(f"[reader] Serial open FEJL: {e}", flush=True)
        return

    print(f"[reader] Port åbnet, drainer startup-buffer...", flush=True)
    drain_end = time.monotonic() + 1.5
    while time.monotonic() < drain_end:
        try:
            if ser.in_waiting:
                ser.read(ser.in_waiting)
        except Exception:
            pass
        time.sleep(0.05)
    ser.reset_input_buffer()
    print(f"[reader] Klar: {port}  FS={FS} Hz", flush=True)

    rx          = bytearray()
    last_seq    = None
    consec_errs = 0

    while not stop_event.is_set():
        try:
            try:
                waiting = ser.in_waiting
            except (serial.SerialException, OSError):
                waiting = 0

            if waiting > 131_072:
                ser.reset_input_buffer()
                rx.clear()
                continue

            chunk = ser.read(max(1, waiting))
            consec_errs = 0

        except (serial.SerialException, OSError) as e:
            consec_errs += 1
            print(f"[reader] Fejl #{consec_errs}: {e}", flush=True)
            if consec_errs >= 10:
                break
            time.sleep(0.2)
            continue

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

            if last_seq is not None:
                gap = (seq - last_seq - 1) & 0xFFFF_FFFF
                if gap:
                    packet_drops += gap
            last_seq = seq

            raw   = np.frombuffer(payload, dtype="<i4") >> 8
            ring_write(raw[0::2], raw[1::2])

    try:
        ser.close()
    except Exception:
        pass
    print("[reader] Stoppet.", flush=True)


# ── FFT ───────────────────────────────────────────────────────────────────
def compute_fft(samples: np.ndarray):
    x  = samples.astype(np.float64) - np.mean(samples)   # AC-kobling
    w  = np.hanning(len(x))
    cg = np.mean(w)                                        # coherent gain
    spec = np.fft.rfft(x * w)
    mag  = np.abs(spec) / (len(x) * cg)
    if len(mag) > 2:
        mag[1:-1] *= 2.0                                   # enkeltsidet
    # dBFS: 0 dBFS = full scale sinus (amplitude = 2^23)
    mag_db = 20 * np.log10(np.maximum(mag / FULL_SCALE, 1e-12))
    freqs  = np.fft.rfftfreq(len(x), d=1.0 / FS)
    mask   = (freqs >= FREQ_MIN) & (freqs <= FREQ_MAX)
    return freqs[mask], mag_db[mask]


# ── Plot setup ────────────────────────────────────────────────────────────
fig, (ax_l, ax_r) = plt.subplots(2, 1, figsize=(13, 8), sharex=True,
                                   gridspec_kw={"hspace": 0.12})
fig.patch.set_facecolor("#1a1a2e")
fig.suptitle(f"Audio Analyzer — FFT Spektrum (24-bit, {FS//1000} kHz)",
             color="white", fontsize=13, fontweight="bold")

_f0 = np.linspace(FREQ_MIN, FREQ_MAX, 512)
_d0 = np.full(512, DB_MIN)

for ax, title, col in [(ax_l, "Kanal L", "#00d4ff"),
                        (ax_r, "Kanal R", "#ff6b6b")]:
    ax.set_facecolor("#0d0d1a")
    ax.set_xlim(FREQ_MIN, FREQ_MAX)
    ax.set_ylim(DB_MIN, DB_MAX)
    ax.set_ylabel("dBFS", color="white", fontsize=10)
    ax.set_title(title, color=col, fontsize=11, loc="left", pad=4)
    ax.tick_params(colors="white", labelsize=9)
    for sp in ax.spines.values():
        sp.set_color("#444466")
    ax.grid(True, color="#2a2a4a", linewidth=0.7, linestyle="--")
    ax.set_yticks(np.arange(DB_MIN, DB_MAX + 1, 10))
    step = (FREQ_MAX - FREQ_MIN) / 6
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

line_l,      = ax_l.plot(_f0, _d0, color="#00d4ff", linewidth=0.9)
fill_l        = ax_l.fill_between(_f0, DB_MIN, _d0, color="#00d4ff", alpha=0.15)
peak_line_l, = ax_l.plot(_f0, _d0, color="#80eeff", linewidth=0.7,
                          linestyle="--", alpha=0.5)
line_r,      = ax_r.plot(_f0, _d0, color="#ff6b6b", linewidth=0.9)
fill_r        = ax_r.fill_between(_f0, DB_MIN, _d0, color="#ff6b6b", alpha=0.15)
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
status_txt  = fig.text(0.01, 0.005, "Status: venter...", color="white", fontsize=8.5)
marker_txt  = fig.text(0.01, 0.030, "", color="#ffee44", fontsize=8.5)

# ── Manuel markør (klik for at placere, højreklik for at fjerne) ──────────
_marker_vline_l = ax_l.axvline(0, color="#ffee44", lw=1.2, ls="--", alpha=0.0)
_marker_vline_r = ax_r.axvline(0, color="#ffee44", lw=1.2, ls="--", alpha=0.0)
_marker_freq    = [None]   # liste så vi kan mutere inde fra closure

# Cache af seneste spektrum til markør-opslag
_last_freqs_l = [_f0]
_last_mag_l   = [_d0]
_last_freqs_r = [_f0]
_last_mag_r   = [_d0]


def _on_click(event):
    if event.inaxes not in (ax_l, ax_r):
        return
    if event.button == 3:              # højreklik = fjern markør
        _marker_freq[0] = None
        _marker_vline_l.set_alpha(0.0)
        _marker_vline_r.set_alpha(0.0)
        marker_txt.set_text("")
        fig.canvas.draw_idle()
        return
    if event.button != 1:
        return

    freq = event.xdata
    if freq is None:
        return

    _marker_freq[0] = freq
    _marker_vline_l.set_xdata([freq, freq])
    _marker_vline_r.set_xdata([freq, freq])
    _marker_vline_l.set_alpha(0.85)
    _marker_vline_r.set_alpha(0.85)

    # Find nærmeste bin og hent amplitude
    def nearest_db(freqs, mag, f):
        if len(freqs) == 0:
            return None
        idx = int(np.argmin(np.abs(freqs - f)))
        return freqs[idx], mag[idx]

    res_l = nearest_db(_last_freqs_l[0], _last_mag_l[0], freq)
    res_r = nearest_db(_last_freqs_r[0], _last_mag_r[0], freq)
    if res_l and res_r:
        marker_txt.set_text(
            f"▶ Markør: {res_l[0]:.1f} Hz  |  "
            f"L: {res_l[1]:.1f} dBFS   R: {res_r[1]:.1f} dBFS"
        )
    fig.canvas.draw_idle()


fig.canvas.mpl_connect("button_press_event", _on_click)

plt.tight_layout(rect=[0, 0.06, 1, 0.95])

avg_l       = collections.deque(maxlen=AVG_FRAMES)
avg_r       = collections.deque(maxlen=AVG_FRAMES)
peak_l_hold = None
peak_r_hold = None


def update(_):
    global fill_l, fill_r, peak_l_hold, peak_r_hold

    avail = ring_available()
    if avail < FFT_SIZE:
        status_txt.set_text(
            f"Status: venter... ({avail}/{FFT_SIZE} samples)"
        )
        return

    L_arr, R_arr = ring_read_last(FFT_SIZE)

    freqs_l, mag_l = compute_fft(L_arr)
    freqs_r, mag_r = compute_fft(R_arr)

    # Midling i lineært domæne
    avg_l.append(10 ** (mag_l / 20))
    avg_r.append(10 ** (mag_r / 20))
    mag_l = 20 * np.log10(np.mean(avg_l, axis=0))
    mag_r = 20 * np.log10(np.mean(avg_r, axis=0))

    # Gem til markør-opslag
    _last_freqs_l[0] = freqs_l
    _last_mag_l[0]   = mag_l
    _last_freqs_r[0] = freqs_r
    _last_mag_r[0]   = mag_r

    # Peak-hold
    if peak_l_hold is None or len(peak_l_hold) != len(mag_l):
        peak_l_hold = mag_l.copy()
        peak_r_hold = mag_r.copy()
    else:
        peak_l_hold = np.maximum(peak_l_hold - PEAK_DECAY, mag_l)
        peak_r_hold = np.maximum(peak_r_hold - PEAK_DECAY, mag_r)

    line_l.set_data(freqs_l, mag_l)
    line_r.set_data(freqs_r, mag_r)
    peak_line_l.set_data(freqs_l, peak_l_hold)
    peak_line_r.set_data(freqs_r, peak_r_hold)

    fill_l.remove()
    fill_r.remove()
    fill_l = ax_l.fill_between(freqs_l, DB_MIN, mag_l, color="#00d4ff", alpha=0.15)
    fill_r = ax_r.fill_between(freqs_r, DB_MIN, mag_r, color="#ff6b6b", alpha=0.15)

    ax_l.set_xlim(FREQ_MIN, FREQ_MAX)
    ax_r.set_xlim(FREQ_MIN, FREQ_MAX)
    ax_l.set_ylim(DB_MIN, DB_MAX)
    ax_r.set_ylim(DB_MIN, DB_MAX)

    idx_l = int(np.argmax(mag_l))
    idx_r = int(np.argmax(mag_r))
    pf_l, pd_l = freqs_l[idx_l], mag_l[idx_l]
    pf_r, pd_r = freqs_r[idx_r], mag_r[idx_r]

    txt_l.set_text(f"Peak: {pf_l:.0f} Hz  {pd_l:.1f} dBFS")
    txt_r.set_text(f"Peak: {pf_r:.0f} Hz  {pd_r:.1f} dBFS")

    # Opdater markør-tekst hvis markør er sat
    if _marker_freq[0] is not None:
        f = _marker_freq[0]
        idx_ml = int(np.argmin(np.abs(freqs_l - f)))
        idx_mr = int(np.argmin(np.abs(freqs_r - f)))
        marker_txt.set_text(
            f"▶ Markør: {freqs_l[idx_ml]:.1f} Hz  |  "
            f"L: {mag_l[idx_ml]:.1f} dBFS   R: {mag_r[idx_mr]:.1f} dBFS"
        )

    status_txt.set_text(
        f"{FREQ_MIN/1000:.0f}–{FREQ_MAX/1000:.0f} kHz  "
        f"RBW={FS/FFT_SIZE:.1f}Hz  |  Drops={packet_drops}  |  Status: live  "
        f"|  Klik=markør  Højreklik=fjern"
    )



def on_close(_):
    stop_event.set()


def main():
    port = PORT or find_port()
    if not port:
        print("FEJL: Ingen Teensy fundet. Sæt PORT manuelt øverst.")
        sys.exit(1)

    print(f"Port:        {port}")
    print(f"FS:          {FS} Hz")
    print(f"FFT:         {FFT_SIZE} punkter  →  {FS/FFT_SIZE:.1f} Hz/bin")
    print(f"Span:        {FREQ_MIN/1000:.1f}–{FREQ_MAX/1000:.1f} kHz")

    threading.Thread(target=serial_reader, args=(port,), daemon=True).start()
    fig.canvas.mpl_connect("close_event", on_close)
    ani = animation.FuncAnimation(fig, update, interval=100,
                                   blit=False, cache_frame_data=False)
    plt.show()
    _ = ani


if __name__ == "__main__":
    main()