"""
Audio Analyzer — Oscilloscope Mode
==================================

Updated version:
- Better readable UI (dark text on light controls)
- Added parse mode selector:
    * interleaved  -> L,R,L,R,...
    * swap         -> R,L,R,L,...
    * mono_left    -> duplicate left into both plots
    * mono_right   -> duplicate right into both plots
- Added L/R correlation in status line
- Added simple warning if one channel is much smaller than the other
"""

import sys
import time
import struct
import threading
from dataclasses import dataclass
from typing import Optional

import numpy as np
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets


# =============================================================================
# Teensy / protocol
# =============================================================================
PORT = None
BAUD = 2_000_000
FS   = 192_000

HEADER      = struct.Struct("<III")
MAGIC       = 0x30445541
MAGIC_BYTES = struct.pack("<I", MAGIC)
MAX_WORDS   = 65536

# Default parser mode
DEFAULT_PARSE_MODE = "interleaved"   # interleaved, swap, mono_left, mono_right


# =============================================================================
# Ring buffer
# =============================================================================
RING_SECS = 1.0
RING_SIZE = int(FS * RING_SECS)

_ring_l    = np.zeros(RING_SIZE, dtype=np.int32)
_ring_r    = np.zeros(RING_SIZE, dtype=np.int32)
_write_ptr = 0
_ptr_lock  = threading.Lock()

packet_drops = 0
stop_event   = threading.Event()


def ring_write(left: np.ndarray, right: np.ndarray):
    global _write_ptr

    n = len(left)
    ptr = _write_ptr % RING_SIZE
    end = ptr + n

    if end <= RING_SIZE:
        _ring_l[ptr:end] = left
        _ring_r[ptr:end] = right
    else:
        first = RING_SIZE - ptr
        _ring_l[ptr:]       = left[:first]
        _ring_l[:n - first] = left[first:]
        _ring_r[ptr:]       = right[:first]
        _ring_r[:n - first] = right[first:]

    with _ptr_lock:
        _write_ptr += n


def ring_available() -> int:
    return min(_write_ptr, RING_SIZE)


def ring_read_last(n: int):
    with _ptr_lock:
        ptr = _write_ptr

    start = (ptr - n) % RING_SIZE
    if start + n <= RING_SIZE:
        return _ring_l[start:start + n].copy(), _ring_r[start:start + n].copy()

    cut = RING_SIZE - start
    l = np.concatenate([_ring_l[start:], _ring_l[:n - cut]])
    r = np.concatenate([_ring_r[start:], _ring_r[:n - cut]])
    return l, r


# =============================================================================
# Display config
# =============================================================================
REFRESH_MS              = 30
DEFAULT_TIME_WINDOW_MS  = 5.0
PRETRIGGER              = 0.25
SEARCH_WINDOWS          = 4
TRIGGER_HOLDOFF_FRAC    = 0.80
TRIGGER_HYSTERESIS_FRAC = 0.05
SHOW_IN_KCOUNTS         = True
DEFAULT_SMOOTH_SAMPLES  = 1


# =============================================================================
# Helpers
# =============================================================================
def find_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        manu = (p.manufacturer or "").lower() if p.manufacturer else ""
        if "teensy" in desc or "pjrc" in manu:
            return p.device

    ports = list(serial.tools.list_ports.comports())
    if len(ports) == 1:
        return ports[0].device
    return None


def counts_to_display(x: np.ndarray) -> np.ndarray:
    return x.astype(np.float64) / 1000.0 if SHOW_IN_KCOUNTS else x.astype(np.float64)


def display_unit() -> str:
    return "kcounts" if SHOW_IN_KCOUNTS else "counts"


def smooth_signal(x: np.ndarray, n: int) -> np.ndarray:
    if n <= 1:
        return x
    kernel = np.ones(n, dtype=np.float64) / float(n)
    return np.convolve(x, kernel, mode="same")


def estimate_freq(x: np.ndarray, fs: float):
    if len(x) < 32:
        return None

    y = x - np.mean(x)
    if np.max(np.abs(y)) < 10:
        return None

    crossings = np.where((y[:-1] < 0) & (y[1:] >= 0))[0]
    if len(crossings) < 2:
        return None

    periods = np.diff(crossings) / fs
    periods = periods[periods > 0]
    return 1.0 / np.median(periods) if len(periods) else None


def basic_stats(x: np.ndarray, fs: float) -> dict:
    xf = x.astype(np.float64)
    return {
        "mean": float(np.mean(xf)),
        "rms":  float(np.sqrt(np.mean(xf * xf))),
        "vpp":  float(np.ptp(xf)),
        "freq": estimate_freq(xf, fs),
    }


def parse_payload(raw: np.ndarray, mode: str):
    if mode == "interleaved":
        left  = raw[0::2]
        right = raw[1::2]
    elif mode == "swap":
        left  = raw[1::2]
        right = raw[0::2]
    elif mode == "mono_left":
        left  = raw[0::2]
        right = left.copy()
    elif mode == "mono_right":
        right = raw[1::2]
        left  = right.copy()
    else:
        left  = raw[0::2]
        right = raw[1::2]

    n = min(len(left), len(right))
    return left[:n], right[:n]


def find_trigger_start(x, window_samples, pretrigger, level, edge, holdoff):
    n = len(x)
    if n < window_samples + 2:
        return 0, False

    pre        = int(window_samples * pretrigger)
    search_len = min(n, window_samples * SEARCH_WINDOWS)
    search     = x[-search_len:]
    s_len      = len(search)

    sig_range = float(np.ptp(search))
    hyst      = max(sig_range * TRIGGER_HYSTERESIS_FRAC, abs(level) * 0.05 + 1.0)

    if edge == "rising":
        armed    = search < (level - hyst)
        crossing = (search[:-1] < level) & (search[1:] >= level)
        arm_before = np.zeros(s_len - 1, dtype=bool)
        state = False
        for i in range(s_len - 1):
            if armed[i]:
                state = True
            if crossing[i]:
                arm_before[i] = state
                state = False
        cands = np.where(arm_before & crossing)[0] + 1
    else:
        armed    = search > (level + hyst)
        crossing = (search[:-1] > level) & (search[1:] <= level)
        arm_before = np.zeros(s_len - 1, dtype=bool)
        state = False
        for i in range(s_len - 1):
            if armed[i]:
                state = True
            if crossing[i]:
                arm_before[i] = state
                state = False
        cands = np.where(arm_before & crossing)[0] + 1

    if len(cands) == 0:
        return n - window_samples, False

    for idx in reversed(cands):
        abs_idx = (n - search_len) + idx
        start   = abs_idx - pre
        end     = start + window_samples
        if start >= 0 and end <= n:
            if abs_idx + holdoff > n - pre:
                continue
            return start, True

    return n - window_samples, False


# =============================================================================
# Serial reader thread
# =============================================================================
current_parse_mode = DEFAULT_PARSE_MODE
parse_mode_lock    = threading.Lock()


def get_parse_mode() -> str:
    with parse_mode_lock:
        return current_parse_mode


def set_parse_mode(mode: str):
    global current_parse_mode
    with parse_mode_lock:
        current_parse_mode = mode


def serial_reader_thread(port: str):
    global packet_drops

    try:
        ser = serial.Serial(
            port,
            BAUD,
            timeout=0.1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
    except serial.SerialException as e:
        print(f"[reader] Serial open ERROR: {e}", flush=True)
        return

    print(f"[reader] Port opened, waiting for USB CDC stabilization...", flush=True)
    time.sleep(1.0)

    try:
        ser.reset_input_buffer()
    except Exception as e:
        print(f"[reader] Warning: reset_input_buffer failed: {e}", flush=True)

    print(f"[reader] Ready: {port}  FS={FS} Hz  ring={RING_SIZE} samples", flush=True)

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
                print(f"[reader] OVERRUN ({waiting} bytes) — flushing", flush=True)
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                rx.clear()
                consec_errs = 0
                continue

            chunk = ser.read(max(1, waiting))
            consec_errs = 0

        except (serial.SerialException, OSError) as e:
            consec_errs += 1
            print(f"[reader] Serial error #{consec_errs}: {e}", flush=True)
            if consec_errs >= 10:
                print("[reader] Too many errors — stopping.", flush=True)
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
            if magic != MAGIC or words == 0 or words > MAX_WORDS or (words % 2):
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

            raw = (np.frombuffer(payload, dtype="<i4").astype(np.int32) >> 8)

            mode = get_parse_mode()
            left, right = parse_payload(raw, mode)

            if len(left) and len(right):
                ring_write(left, right)

    try:
        ser.close()
    except Exception:
        pass

    print("[reader] Stopped.", flush=True)


# =============================================================================
# GUI config
# =============================================================================
@dataclass
class ScopeConfig:
    time_window_ms:       float = DEFAULT_TIME_WINDOW_MS
    trigger_enabled:      bool  = True
    trigger_channel:      str   = "L"
    trigger_edge:         str   = "rising"
    trigger_level_counts: float = 0.0
    ac_coupled:           bool  = True
    autoscale:            bool  = True
    smooth_samples:       int   = DEFAULT_SMOOTH_SAMPLES
    parse_mode:           str   = DEFAULT_PARSE_MODE


# =============================================================================
# Main window
# =============================================================================
class ScopeWindow(QtWidgets.QMainWindow):
    def __init__(self, cfg: ScopeConfig):
        super().__init__()
        self.cfg = cfg
        self._last_trigger_start: Optional[int] = None

        self.setWindowTitle("Audio Analyzer — Oscilloscope Mode")
        self.resize(1450, 920)

        pg.setConfigOptions(antialias=False, background="#0a0f1a", foreground="w")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # ── Controls ────────────────────────────────────────────────────────
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)

        self.freeze_btn = QtWidgets.QPushButton("Freeze")
        self.freeze_btn.setCheckable(True)
        controls.addWidget(self.freeze_btn)

        controls.addWidget(QtWidgets.QLabel("Time window (ms):"))
        self.time_box = QtWidgets.QDoubleSpinBox()
        self.time_box.setRange(0.1, 500.0)
        self.time_box.setDecimals(2)
        self.time_box.setValue(cfg.time_window_ms)
        self.time_box.setSingleStep(0.5)
        controls.addWidget(self.time_box)

        self.ac_box = QtWidgets.QCheckBox("AC couple")
        self.ac_box.setChecked(cfg.ac_coupled)
        controls.addWidget(self.ac_box)

        self.autoscale_box = QtWidgets.QCheckBox("Autoscale")
        self.autoscale_box.setChecked(cfg.autoscale)
        controls.addWidget(self.autoscale_box)

        self.trigger_enable_box = QtWidgets.QCheckBox("Trigger")
        self.trigger_enable_box.setChecked(cfg.trigger_enabled)
        controls.addWidget(self.trigger_enable_box)

        controls.addWidget(QtWidgets.QLabel("Trigger ch:"))
        self.trigger_ch_box = QtWidgets.QComboBox()
        self.trigger_ch_box.addItems(["L", "R"])
        self.trigger_ch_box.setCurrentText(cfg.trigger_channel)
        controls.addWidget(self.trigger_ch_box)

        controls.addWidget(QtWidgets.QLabel("Edge:"))
        self.edge_box = QtWidgets.QComboBox()
        self.edge_box.addItems(["rising", "falling"])
        self.edge_box.setCurrentText(cfg.trigger_edge)
        controls.addWidget(self.edge_box)

        controls.addWidget(QtWidgets.QLabel("Level (counts):"))
        self.level_box = QtWidgets.QDoubleSpinBox()
        self.level_box.setRange(-8_388_608, 8_388_607)
        self.level_box.setDecimals(0)
        self.level_box.setSingleStep(1000)
        self.level_box.setValue(cfg.trigger_level_counts)
        controls.addWidget(self.level_box)

        controls.addWidget(QtWidgets.QLabel("Smooth:"))
        self.smooth_box = QtWidgets.QSpinBox()
        self.smooth_box.setRange(1, 64)
        self.smooth_box.setValue(cfg.smooth_samples)
        controls.addWidget(self.smooth_box)

        controls.addWidget(QtWidgets.QLabel("Parse mode:"))
        self.parse_box = QtWidgets.QComboBox()
        self.parse_box.addItems(["interleaved", "swap", "mono_left", "mono_right"])
        self.parse_box.setCurrentText(cfg.parse_mode)
        controls.addWidget(self.parse_box)

        controls.addStretch()

        # ── Plots ───────────────────────────────────────────────────────────
        self.graphics = pg.GraphicsLayoutWidget()
        layout.addWidget(self.graphics, 1)

        self.plot_l = self.graphics.addPlot(row=0, col=0)
        self.plot_r = self.graphics.addPlot(row=1, col=0)

        for plot, title in ((self.plot_l, "Channel L"), (self.plot_r, "Channel R")):
            plot.showGrid(x=True, y=True, alpha=0.25)
            plot.setLabel("left", display_unit())
            plot.setTitle(title)
            plot.setMenuEnabled(False)
            plot.setMouseEnabled(x=True, y=True)

        self.plot_r.setLabel("bottom", "Time", units="ms")
        self.plot_l.setXLink(self.plot_r)

        self.curve_l = self.plot_l.plot(pen=pg.mkPen("#00d4ff", width=1.5))
        self.curve_r = self.plot_r.plot(pen=pg.mkPen("#ff6b6b", width=1.5))

        dot = QtCore.Qt.PenStyle.DotLine
        dsh = QtCore.Qt.PenStyle.DashLine

        self.zero_line_l  = pg.InfiniteLine(angle=0,  pen=pg.mkPen("#666666", width=1, style=dot))
        self.zero_line_r  = pg.InfiniteLine(angle=0,  pen=pg.mkPen("#666666", width=1, style=dot))
        self.trig_line_l  = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ffaa00", width=1, style=dsh))
        self.trig_line_r  = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ffaa00", width=1, style=dsh))
        self.level_line_l = pg.InfiniteLine(angle=0,  pen=pg.mkPen("#aaaaaa", width=1, style=dot))
        self.level_line_r = pg.InfiniteLine(angle=0,  pen=pg.mkPen("#aaaaaa", width=1, style=dot))

        for item, plot in (
            (self.zero_line_l,  self.plot_l),
            (self.zero_line_r,  self.plot_r),
            (self.trig_line_l,  self.plot_l),
            (self.trig_line_r,  self.plot_r),
            (self.level_line_l, self.plot_l),
            (self.level_line_r, self.plot_r),
        ):
            plot.addItem(item)

        self.status = QtWidgets.QLabel("Waiting for data...")
        self.status.setWordWrap(True)
        self.status.setStyleSheet("""
            QLabel {
                color: #111111;
                background-color: #e9e9e9;
                border: 1px solid #b0b0b0;
                padding: 8px;
                font-family: Consolas, 'Courier New', monospace;
                font-size: 10pt;
            }
        """)
        layout.addWidget(self.status)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_scope)
        self.timer.start(REFRESH_MS)

    def closeEvent(self, event):
        stop_event.set()
        super().closeEvent(event)

    def update_scope(self):
        if self.freeze_btn.isChecked():
            return

        # Read widget config
        self.cfg.time_window_ms       = self.time_box.value()
        self.cfg.ac_coupled           = self.ac_box.isChecked()
        self.cfg.autoscale            = self.autoscale_box.isChecked()
        self.cfg.trigger_enabled      = self.trigger_enable_box.isChecked()
        self.cfg.trigger_channel      = self.trigger_ch_box.currentText()
        self.cfg.trigger_edge         = self.edge_box.currentText()
        self.cfg.trigger_level_counts = self.level_box.value()
        self.cfg.smooth_samples       = self.smooth_box.value()
        self.cfg.parse_mode           = self.parse_box.currentText()

        set_parse_mode(self.cfg.parse_mode)

        window_samples = max(128, int(FS * self.cfg.time_window_ms / 1000.0))
        needed         = max(window_samples * SEARCH_WINDOWS, window_samples + 64)

        avail = ring_available()
        if avail < needed:
            self.status.setText(
                f"Waiting for data...  have={avail}  need={needed}  "
                f"({avail / FS * 1000:.0f} ms / {needed / FS * 1000:.0f} ms)"
            )
            return

        l_raw, r_raw = ring_read_last(needed)

        l = l_raw.astype(np.float64)
        r = r_raw.astype(np.float64)

        if self.cfg.ac_coupled:
            l -= np.mean(l)
            r -= np.mean(r)

        trig_src   = l if self.cfg.trigger_channel == "L" else r
        trig_level = float(self.cfg.trigger_level_counts)

        if self.cfg.trigger_enabled:
            holdoff = max(32, int(window_samples * TRIGGER_HOLDOFF_FRAC))
            start, triggered = find_trigger_start(
                trig_src,
                window_samples,
                PRETRIGGER,
                trig_level,
                self.cfg.trigger_edge,
                holdoff,
            )

            if triggered:
                self._last_trigger_start = start
            elif self._last_trigger_start is not None:
                start     = self._last_trigger_start
                triggered = False
        else:
            start, triggered = len(l) - window_samples, False

        end = start + window_samples
        if start < 0 or end > len(l):
            start = max(0, len(l) - window_samples)
            end   = start + window_samples

        l_win = l[start:end]
        r_win = r[start:end]

        if len(l_win) < 16 or len(r_win) < 16:
            return

        l_disp = counts_to_display(smooth_signal(l_win, self.cfg.smooth_samples))
        r_disp = counts_to_display(smooth_signal(r_win, self.cfg.smooth_samples))

        t_ms         = np.arange(len(l_disp)) * (1000.0 / FS)
        trigger_x_ms = PRETRIGGER * self.cfg.time_window_ms

        self.curve_l.setData(t_ms, l_disp)
        self.curve_r.setData(t_ms, r_disp)

        self.plot_l.setXRange(0, self.cfg.time_window_ms, padding=0)
        self.plot_r.setXRange(0, self.cfg.time_window_ms, padding=0)

        self.zero_line_l.setPos(0)
        self.zero_line_r.setPos(0)

        lvl_disp = trig_level / 1000.0 if SHOW_IN_KCOUNTS else trig_level
        self.level_line_l.setPos(lvl_disp)
        self.level_line_r.setPos(lvl_disp)
        self.level_line_l.setVisible(self.cfg.trigger_enabled)
        self.level_line_r.setVisible(self.cfg.trigger_enabled)

        self.trig_line_l.setPos(trigger_x_ms)
        self.trig_line_r.setPos(trigger_x_ms)
        self.trig_line_l.setVisible(self.cfg.trigger_enabled)
        self.trig_line_r.setVisible(self.cfg.trigger_enabled)

        if self.cfg.autoscale:
            for plot, data in ((self.plot_l, l_disp), (self.plot_r, r_disp)):
                mn   = float(np.min(data))
                mx   = float(np.max(data))
                span = max(mx - mn, 1.0 if SHOW_IN_KCOUNTS else 100.0)
                mg   = span * 0.18
                plot.setYRange(mn - mg, mx + mg, padding=0)

        sl = basic_stats(l_win, FS)
        sr = basic_stats(r_win, FS)

        try:
            corr = float(np.corrcoef(l_win, r_win)[0, 1]) if len(l_win) > 8 else float("nan")
        except Exception:
            corr = float("nan")

        def ff(f):
            return f"{f:,.1f} Hz" if f else "n/a"

        warn = ""
        if sl["vpp"] > 0:
            ratio = sr["vpp"] / sl["vpp"]
            if ratio < 0.2:
                warn = " | WARNING: R much smaller than L"
        if sr["vpp"] > 0:
            ratio2 = sl["vpp"] / sr["vpp"]
            if ratio2 < 0.2:
                warn = " | WARNING: L much smaller than R"

        self.status.setText(
            f"Trigger: {'ON' if self.cfg.trigger_enabled else 'OFF'}"
            f" | Ch={self.cfg.trigger_channel}"
            f" | Edge={self.cfg.trigger_edge}"
            f" | Level={int(trig_level)} counts"
            f" | Window={self.cfg.time_window_ms:.2f} ms"
            f" | Drops={packet_drops}"
            f" | FS={FS} Hz"
            f" | AC={'ON' if self.cfg.ac_coupled else 'OFF'}"
            f" | Smooth={self.cfg.smooth_samples}"
            f" | Parse={self.cfg.parse_mode}"
            f"{warn}"
            f"\nL: mean={sl['mean']:.0f}  rms={sl['rms']:.0f}  vpp={sl['vpp']:.0f} counts  freq≈{ff(sl['freq'])}"
            f"\nR: mean={sr['mean']:.0f}  rms={sr['rms']:.0f}  vpp={sr['vpp']:.0f} counts  freq≈{ff(sr['freq'])}"
            f"\nCorr(L,R)={corr:.3f}"
            f"\nState: {'triggered' if triggered else 'free-run'}"
        )


# =============================================================================
# Main
# =============================================================================
def main():
    port = PORT or find_port()
    if not port:
        print("ERROR: No Teensy found. Set PORT manually at the top of the script.")
        sys.exit(1)

    print(f"Port:        {port}")
    print(f"Sample rate: {FS} Hz")
    print(f"Ring buffer: {RING_SIZE} samples ({RING_SECS:.1f} s)")

    reader = threading.Thread(
        target=serial_reader_thread,
        args=(port,),
        daemon=True,
    )
    reader.start()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet("""
        QWidget {
            font-size: 11pt;
        }

        QMainWindow, QWidget {
            background-color: #dcdcdc;
        }

        QLabel {
            color: #111111;
            background: transparent;
        }

        QCheckBox {
            color: #111111;
            spacing: 6px;
            background: transparent;
        }

        QPushButton {
            color: #111111;
            background-color: #d8e6f5;
            border: 1px solid #6a8fb5;
            border-radius: 4px;
            padding: 6px 10px;
            min-height: 24px;
        }

        QPushButton:checked {
            background-color: #9fc7ea;
        }

        QComboBox, QSpinBox, QDoubleSpinBox {
            color: #111111;
            background-color: #ffffff;
            border: 1px solid #888888;
            border-radius: 4px;
            padding: 4px 6px;
            min-height: 24px;
        }
    """)

    win = ScopeWindow(ScopeConfig())
    win.show()
    app.exec()

    stop_event.set()
    reader.join(timeout=3.0)
    print("Done.")


if __name__ == "__main__":
    main()