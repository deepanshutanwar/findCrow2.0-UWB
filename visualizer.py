"""
UWB Tag Live Visualizer  — v5 (smooth edition)
==================================================
Smoothing stack (applied in order):
  1. Median pre-filter on incoming distances (window=5) — kills single-sample spikes
  2. Distance outlier gate — ignores readings >DIST_OUTLIER_GATE metres from the rolling median
  3. 2-D Kalman filter on the computed position — optimal for noisy sensor tracking
  4. Velocity cap — prevents teleport jumps from bad trilateration solutions

On launch, asks for the 3 anchor positions before starting.
Press Enter on each field, then click Start.

Requirements:
    pip install matplotlib numpy
Usage:
    python visualizer.py
"""

import socket
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.widgets as widgets
from matplotlib.animation import FuncAnimation
from collections import deque

# ─── Defaults (pre-filled in the calibration screen) ─────────────────────────
DEFAULT_ANCHORS = {
    1: (0.0, 0.0),
    2: (3.5, 0.0),
    3: (0.0, 3.5),
}

UDP_PORT     = 5005
TRAIL_LENGTH = 120
PAD          = 0.0

ANCHOR_COLORS = {1: "#e74c3c", 2: "#3498db", 3: "#2ecc71"}
TAG_COLOR     = "#f39c12"

UPDATE_MS = 50   # animation refresh interval (ms)

# ─── Smoothing knobs ──────────────────────────────────────────────────────────
# 1. Median pre-filter window for each anchor's distance history
MEDIAN_WINDOW = 5          # keep last N readings, use median for trilateration

# 2. Distance outlier gate (applied after median; in metres)
DIST_OUTLIER_GATE = 1.5    # reject a distance if it's >this many metres from
                           # the running median — catches big NLOS spikes

# 3. Kalman filter process/measurement noise
#    Q: process noise covariance — lower = trust model more (less jitter)
#       but slower to follow fast movement. Try 0.005–0.05.
#    R: measurement noise covariance — higher = trust measurements less.
#       Try 0.5–3.0 depending on how noisy your env is.
KALMAN_Q = 0.01    # process noise
KALMAN_R = 1.0     # measurement noise

# 4. Maximum plausible speed of the tag (m/s).
#    Jumps larger than this × dt are capped to this speed.
MAX_SPEED_M_S = 2.0   # ~fast walking; lower for slow-moving objects

# ─── Will be filled after calibration ────────────────────────────────────────
ANCHOR_POSITIONS = {}

# ─── Shared state ─────────────────────────────────────────────────────────────
state = {
    "distances":    {1: None, 2: None, 3: None},
    "tag_pos":      None,   # raw trilaterated position
    "smoothed_pos": None,   # Kalman-filtered position
    "trail":        deque(maxlen=TRAIL_LENGTH),
    "last_update":  0.0,
    "packets":      0,
}
lock = threading.Lock()

# Per-anchor distance history for median filter
dist_history = {1: deque(maxlen=MEDIAN_WINDOW),
                2: deque(maxlen=MEDIAN_WINDOW),
                3: deque(maxlen=MEDIAN_WINDOW)}


# ─── 2-D Kalman Filter ────────────────────────────────────────────────────────
# State vector: [x, y, vx, vy]
# Constant-velocity model.

class KalmanFilter2D:
    def __init__(self, q=KALMAN_Q, r=KALMAN_R):
        self.initialized = False
        # State estimate [x, y, vx, vy]
        self.x = np.zeros(4)
        # State covariance
        self.P = np.eye(4) * 1.0
        # Transition matrix (updated every step with actual dt)
        self.F = np.eye(4)
        # Measurement matrix (we observe x, y only)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=float)
        # Process noise covariance
        self.Q_base = q
        # Measurement noise covariance
        self.R = np.eye(2) * r

    def _build_F(self, dt):
        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt
        return F

    def _build_Q(self, dt):
        # Discretised white noise acceleration model
        q = self.Q_base
        dt2, dt3, dt4 = dt**2, dt**3, dt**4
        Q = np.array([
            [dt4/4, 0,     dt3/2, 0    ],
            [0,     dt4/4, 0,     dt3/2],
            [dt3/2, 0,     dt2,   0    ],
            [0,     dt3/2, 0,     dt2  ],
        ]) * q
        return Q

    def update(self, meas_x, meas_y, dt):
        """Feed a new measurement; returns (filtered_x, filtered_y)."""
        z = np.array([meas_x, meas_y])

        if not self.initialized:
            self.x = np.array([meas_x, meas_y, 0.0, 0.0])
            self.initialized = True
            return meas_x, meas_y

        dt = max(dt, 1e-3)
        F  = self._build_F(dt)
        Q  = self._build_Q(dt)

        # Predict
        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + Q

        # Update
        y_res  = z - self.H @ x_pred
        S      = self.H @ P_pred @ self.H.T + self.R
        K      = P_pred @ self.H.T @ np.linalg.inv(S)
        self.x = x_pred + K @ y_res
        self.P = (np.eye(4) - K @ self.H) @ P_pred

        return float(self.x[0]), float(self.x[1])


kalman = KalmanFilter2D()
last_kalman_time = None   # wall-clock time of last Kalman update


# ─── Trilateration ────────────────────────────────────────────────────────────

def trilaterate(d1, d2, d3):
    (x1, y1) = ANCHOR_POSITIONS[1]
    (x2, y2) = ANCHOR_POSITIONS[2]
    (x3, y3) = ANCHOR_POSITIONS[3]
    A = np.array([
        [2*(x2-x1), 2*(y2-y1)],
        [2*(x3-x1), 2*(y3-y1)],
    ])
    b = np.array([
        d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2,
        d1**2 - d3**2 - x1**2 + x3**2 - y1**2 + y3**2,
    ])
    try:
        if abs(np.linalg.det(A)) < 1e-10:
            return None
        pos = np.linalg.solve(A, b)
        return float(pos[0]), float(pos[1])
    except Exception:
        return None


def median_distances():
    """Return median-filtered distances for each anchor (or None if not enough data)."""
    result = {}
    for aid in (1, 2, 3):
        h = list(dist_history[aid])
        if len(h) == 0:
            result[aid] = None
        else:
            med = float(np.median(h))
            # Gate: reject latest sample if it's far from the median
            if len(h) >= 2 and abs(h[-1] - med) > DIST_OUTLIER_GATE:
                # Use median of the rest (without the outlier)
                med = float(np.median(h[:-1]))
            result[aid] = med
    return result


# ─── UDP listener ─────────────────────────────────────────────────────────────

def udp_listener():
    global last_kalman_time

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(1.0)
    print(f"[UDP] Listening on 0.0.0.0:{UDP_PORT} ...")

    while True:
        try:
            data, _ = sock.recvfrom(256)
            msg = data.decode("utf-8").strip()

            if msg.startswith("TAG:"):
                parts = msg[4:].split(",")
                if len(parts) == 3:
                    try:
                        d1, d2, d3 = float(parts[0]), float(parts[1]), float(parts[2])

                        # Feed raw distances into per-anchor history
                        for aid, d in zip((1, 2, 3), (d1, d2, d3)):
                            if d > 0:
                                dist_history[aid].append(d)

                        # Get median-filtered distances
                        meds = median_distances()
                        md1, md2, md3 = meds[1], meds[2], meds[3]

                        if md1 is not None and md2 is not None and md3 is not None:
                            pos = trilaterate(md1, md2, md3)
                        else:
                            pos = None

                        with lock:
                            state["distances"][1] = d1
                            state["distances"][2] = d2
                            state["distances"][3] = d3
                            state["packets"] += 1
                            _print_raw = None
                            _print_sp  = None

                            if pos:
                                state["tag_pos"] = pos

                                # Velocity cap — prevent teleport jumps
                                now = time.time()
                                dt  = (now - last_kalman_time) if last_kalman_time else 0.2
                                last_kalman_time = now

                                prev = state["smoothed_pos"]
                                if prev is not None:
                                    dx = pos[0] - prev[0]
                                    dy = pos[1] - prev[1]
                                    dist_step = np.hypot(dx, dy)
                                    max_step  = MAX_SPEED_M_S * dt
                                    if dist_step > max_step and dist_step > 0:
                                        # Clamp the raw measurement toward prev
                                        scale = max_step / dist_step
                                        pos = (prev[0] + dx * scale,
                                               prev[1] + dy * scale)

                                # Kalman update
                                kx, ky = kalman.update(pos[0], pos[1], dt)
                                state["smoothed_pos"] = (kx, ky)
                                state["trail"].append(state["smoothed_pos"])
                                state["last_update"] = time.time()
                                # capture inside lock so print below is race-free
                                _print_raw = pos
                                _print_sp  = state["smoothed_pos"]
                            else:
                                _print_raw = None
                                _print_sp  = None

                        if _print_raw:
                            print(f"  TAG raw=({_print_raw[0]:.2f},{_print_raw[1]:.2f}) "
                                  f"kalman=({_print_sp[0]:.2f},{_print_sp[1]:.2f}) m")

                    except ValueError:
                        pass

            elif msg.startswith("A") and ":" in msg:
                try:
                    aid  = int(msg[1])
                    dist = float(msg.split(":")[1])
                    if aid in (1, 2, 3) and dist > 0:
                        dist_history[aid].append(dist)
                        with lock:
                            state["distances"][aid] = dist
                except (ValueError, IndexError):
                    pass

        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP error] {e}")


# ─── Calibration screen ───────────────────────────────────────────────────────

def show_calibration():
    fig_cal = plt.figure(figsize=(6, 6))
    fig_cal.patch.set_facecolor("#1a1a2e")
    fig_cal.canvas.manager.set_window_title("UWB Visualizer — Calibration")

    ax = fig_cal.add_axes([0, 0, 1, 1])
    ax.set_facecolor("#1a1a2e")
    ax.axis("off")

    ax.text(0.5, 0.93, "UWB Anchor Calibration",
            transform=ax.transAxes, color="white",
            fontsize=15, fontweight="bold", ha="center", va="top")
    ax.text(0.5, 0.87,
            "Enter each anchor's position (metres from your room corner).\n"
            "A1 is always the origin — keep it at 0, 0.",
            transform=ax.transAxes, color="#aaaaaa",
            fontsize=9, ha="center", va="top")

    for label, xpos in [("Anchor", 0.08), ("X (m)", 0.42), ("Y (m)", 0.68)]:
        ax.text(xpos, 0.76, label,
                transform=ax.transAxes, color="#888888",
                fontsize=9, va="top")

    box_specs = {
        1: (0.40, 0.66, 0.66, 0.66),
        2: (0.40, 0.54, 0.66, 0.54),
        3: (0.40, 0.42, 0.66, 0.42),
    }

    text_boxes = {}
    w, h = 0.20, 0.06

    for aid in (1, 2, 3):
        xl, yl, xr, yr = box_specs[aid]

        ax.text(0.08, yl + 0.03, f"Anchor {aid}",
                transform=ax.transAxes, color=ANCHOR_COLORS[aid],
                fontsize=11, va="center", fontweight="bold")

        ax_xbox = fig_cal.add_axes([xl, yl, w, h])
        tb_x = widgets.TextBox(ax_xbox, "", initial=str(DEFAULT_ANCHORS[aid][0]),
                               color="#16213e", hovercolor="#1e2d4a",
                               label_pad=0.01)
        tb_x.label.set_color("white")
        tb_x.text_disp.set_color("white")

        ax_ybox = fig_cal.add_axes([xr, yr, w, h])
        tb_y = widgets.TextBox(ax_ybox, "", initial=str(DEFAULT_ANCHORS[aid][1]),
                               color="#16213e", hovercolor="#1e2d4a",
                               label_pad=0.01)
        tb_y.label.set_color("white")
        tb_y.text_disp.set_color("white")

        text_boxes[aid] = (tb_x, tb_y)

    ax.axhline(0.35, color="#333355", linewidth=0.8, xmin=0.05, xmax=0.95)

    ax_preview = fig_cal.add_axes([0.1, 0.09, 0.80, 0.22])
    ax_preview.set_facecolor("#16213e")
    ax_preview.tick_params(colors="#555566", labelsize=7)
    for spine in ax_preview.spines.values():
        spine.set_edgecolor("#333355")
    ax_preview.set_title("Anchor layout preview", color="#888888",
                          fontsize=8, pad=4)
    preview_dots, preview_labels = {}, {}
    for aid in (1, 2, 3):
        x, y = DEFAULT_ANCHORS[aid]
        dot, = ax_preview.plot(x, y, "^", markersize=10,
                               color=ANCHOR_COLORS[aid],
                               markeredgecolor="white", markeredgewidth=0.8)
        lbl  = ax_preview.annotate(f" A{aid}", (x, y),
                                   color=ANCHOR_COLORS[aid], fontsize=7)
        preview_dots[aid]   = dot
        preview_labels[aid] = lbl
    ax_preview.set_xlim(-1, 11)
    ax_preview.set_ylim(-1, 11)
    ax_preview.grid(True, color="#2a2a4a", linewidth=0.4)

    result = {"started": False}

    def refresh_preview(_=None):
        for aid in (1, 2, 3):
            try:
                x = float(text_boxes[aid][0].text)
                y = float(text_boxes[aid][1].text)
                preview_dots[aid].set_data([x], [y])
                preview_labels[aid].set_position((x, y))
            except ValueError:
                pass
        fig_cal.canvas.draw_idle()

    for aid in (1, 2, 3):
        text_boxes[aid][0].on_submit(refresh_preview)
        text_boxes[aid][1].on_submit(refresh_preview)

    ax_btn = fig_cal.add_axes([0.35, 0.01, 0.30, 0.07])
    btn_start = widgets.Button(ax_btn, "▶  Start Visualizer",
                               color="#1a6b3a", hovercolor="#218a4a")
    btn_start.label.set_color("white")
    btn_start.label.set_fontsize(10)

    def on_start(_):
        ok = True
        for aid in (1, 2, 3):
            try:
                x = float(text_boxes[aid][0].text)
                y = float(text_boxes[aid][1].text)
                ANCHOR_POSITIONS[aid] = (x, y)
            except ValueError:
                print(f"[Calibration] Invalid value for Anchor {aid}")
                ok = False
        if ok:
            result["started"] = True
            plt.close(fig_cal)

    btn_start.on_clicked(on_start)
    plt.show()
    return result["started"]


# ─── Live plot ────────────────────────────────────────────────────────────────

def show_visualizer():
    _ax = [p[0] for p in ANCHOR_POSITIONS.values()]
    _ay = [p[1] for p in ANCHOR_POSITIONS.values()]

    X_MIN, X_MAX = min(_ax) - PAD, max(_ax) + PAD
    Y_MIN, Y_MAX = min(_ay) - PAD, max(_ay) + PAD
    ROOM_XLIM = (X_MIN, X_MAX)
    ROOM_YLIM = (Y_MIN, Y_MAX)

    fig, ax = plt.subplots(figsize=(9, 7))
    fig.patch.set_facecolor("#1a1a2e")
    fig.canvas.manager.set_window_title("UWB Tag — Live Position (smooth)")
    ax.set_facecolor("#16213e")
    ax.set_xlim(*ROOM_XLIM)
    ax.set_ylim(*ROOM_YLIM)
    ax.set_xlabel("X (metres)", color="white")
    ax.set_ylabel("Y (metres)", color="white")
    ax.tick_params(colors="white")
    ax.set_title("UWB Tag — Live Position  [Kalman + Median filter]",
                 color="white", fontsize=14, pad=10)
    for spine in ax.spines.values():
        spine.set_edgecolor("#444")
    ax.grid(True, color="#2a2a4a", linewidth=0.5)

    room_rect = plt.Rectangle(
        (min(_ax), min(_ay)), max(_ax)-min(_ax), max(_ay)-min(_ay),
        linewidth=1.5, edgecolor="#4a4a8a", facecolor="#1c2340",
        linestyle="-", zorder=1
    )
    ax.add_patch(room_rect)

    for aid, (ax_, ay_) in ANCHOR_POSITIONS.items():
        ax.plot(ax_, ay_, "^", markersize=15, color=ANCHOR_COLORS[aid],
                markeredgecolor="white", markeredgewidth=1.2, zorder=5)
        ax.annotate(f"  A{aid}  ({ax_:.2f}, {ay_:.2f} m)",
                    (ax_, ay_), color=ANCHOR_COLORS[aid], fontsize=9, zorder=6)

    trail_line, = ax.plot([], [], "-", color=TAG_COLOR, alpha=0.35,
                          linewidth=2, zorder=3)
    tag_dot,    = ax.plot([], [], "o", color=TAG_COLOR, markersize=14,
                          markeredgecolor="white", markeredgewidth=1.5, zorder=10)
    tag_label   = ax.annotate("", xy=(0, 0), color="white", fontsize=9,
                               xytext=(12, 6), textcoords="offset points", zorder=11)

    circles = {}
    for aid in (1, 2, 3):
        c = plt.Circle(ANCHOR_POSITIONS[aid], 0, fill=False,
                       color=ANCHOR_COLORS[aid], linestyle="--",
                       linewidth=1.0, alpha=0.4, zorder=2)
        ax.add_patch(c)
        circles[aid] = c

    status = ax.text(0.02, 0.97, "Waiting for UDP packets…",
                     transform=ax.transAxes, color="#aaaaaa",
                     fontsize=9, va="top", family="monospace")

    patches = [mpatches.Patch(color=ANCHOR_COLORS[i], label=f"Anchor {i}") for i in (1, 2, 3)]
    patches.append(mpatches.Patch(color=TAG_COLOR, label="Tag (Kalman)"))
    ax.legend(handles=patches, loc="lower right",
              facecolor="#1a1a2e", edgecolor="#444",
              labelcolor="white", fontsize=9)

    plt.tight_layout()

    def update(_frame):
        with lock:
            pos     = state["smoothed_pos"]
            raw_pos = state["tag_pos"]
            dists   = dict(state["distances"])
            trail   = list(state["trail"])
            age     = time.time() - state["last_update"]
            packets = state["packets"]

        if len(trail) > 1:
            xs, ys = zip(*trail)
            trail_line.set_data(xs, ys)
        else:
            trail_line.set_data([], [])

        if pos:
            tag_dot.set_data([pos[0]], [pos[1]])
            tag_dot.set_alpha(1.0 if age < 2.0 else max(0.2, 1.0 - (age - 2) * 0.15))
            tag_label.set_position((pos[0], pos[1]))
            tag_label.set_text(f"  ({pos[0]:.2f}, {pos[1]:.2f}) m")
        else:
            tag_dot.set_data([], [])
            tag_label.set_text("")

        ax.set_xlim(*ROOM_XLIM)
        ax.set_ylim(*ROOM_YLIM)

        # Show median-filtered distances on the circles
        meds = median_distances()
        for aid in (1, 2, 3):
            r = meds[aid] if meds[aid] else 0
            circles[aid].set_radius(r if r > 0 else 0)

        d1s = f"{dists[1]:.2f}" if dists[1] else "---"
        d2s = f"{dists[2]:.2f}" if dists[2] else "---"
        d3s = f"{dists[3]:.2f}" if dists[3] else "---"
        pos_str = f"({pos[0]:.2f}, {pos[1]:.2f}) m" if pos else "computing…"
        raw_str = f"({raw_pos[0]:.2f}, {raw_pos[1]:.2f}) m" if raw_pos else "---"
        status.set_text(
            f"d1={d1s}m   d2={d2s}m   d3={d3s}m\n"
            f"Position (Kalman) : {pos_str}\n"
            f"Position (raw)    : {raw_str}\n"
            f"Packets: {packets}    Last update: {age:.1f}s ago"
        )

        return trail_line, tag_dot, tag_label, status, *circles.values()

    ani = FuncAnimation(fig, update, interval=UPDATE_MS,
                        blit=True, cache_frame_data=False)
    plt.show()
    print("\n[Done] Window closed.")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    started = show_calibration()
    if not started:
        print("Calibration cancelled — exiting.")
        return

    print("\n=== Calibration done ===")
    for aid, pos in ANCHOR_POSITIONS.items():
        print(f"  Anchor {aid}: ({pos[0]:.2f}, {pos[1]:.2f}) m")

    t = threading.Thread(target=udp_listener, daemon=True)
    t.start()

    show_visualizer()


if __name__ == "__main__":
    main()