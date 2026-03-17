import cv2
import numpy as np
import math
import time
import threading

import matplotlib.pyplot as plt
from pyzbar.pyzbar import decode as pyzbar_decode


class CameraGrabber:
    """Background thread that continuously grabs the newest camera frame.

    cv2.VideoCapture buffers several frames internally.  When the processing
    loop is slower than the camera FPS, cap.read() returns a stale buffered
    frame instead of the latest one.  This class calls cap.grab() in a tight
    loop so that cap.retrieve() always decodes the most recent image.
    """

    def __init__(self, cap):
        self._cap = cap
        self._event = threading.Event()
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._running:
            self._cap.grab()
            self._event.set()

    def retrieve(self):
        self._event.wait()
        self._event.clear()
        return self._cap.retrieve()

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)

# ==========================================
# CONFIGURATION (Fill these in before running)
CAMERA_INDEX = 0                   # Webcam index (0 = built-in, 1+ = USB)
QR_SIZE_MM   = 50.0                # Physical side length of each QR code (mm)
SETPOINT_MM  = 300.0               # Target distance from wall (mm)
CALIB_FILE   = "camera_calib.npz"  # Camera calibration file path
# ==========================================

# Expected QR code text content for each marker
QR_LABEL_CAR  = "CAR"
QR_LABEL_WALL = "WALL"


def process_frame_for_distance(frame, mtx, dist_coeffs, qr_physical_size_mm=50.0, offset=0.0):
    """Detect the CAR and WALL QR codes, compute the real-world distance
    between their centres, and return the annotated frame plus the distance.

    Returns (annotated_frame, distance_mm).  distance_mm is None when the
    expected QR pair is not detected.

    Uses pyzbar for robust QR decoding (OpenCV's built-in QRCodeDetector is
    unreliable with small or simple QR codes in many OpenCV 4.x versions).
    """
    if mtx is not None and dist_coeffs is not None:
        frame = cv2.undistort(frame, mtx, dist_coeffs, None, mtx)

    detected = pyzbar_decode(frame)

    qr_map = {}
    for obj in detected:
        label = obj.data.decode("utf-8", errors="ignore").strip()
        if label in (QR_LABEL_CAR, QR_LABEL_WALL):
            qr_map[label] = obj

    if QR_LABEL_CAR not in qr_map or QR_LABEL_WALL not in qr_map:
        return frame, None

    centers = {}
    scale_factors = []

    for tag in (QR_LABEL_CAR, QR_LABEL_WALL):
        obj = qr_map[tag]
        pts = np.array([(p.x, p.y) for p in obj.polygon], dtype=np.int32)
        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        centers[tag] = (cx, cy)

        color = (0, 200, 255) if tag == "CAR" else (0, 255, 0)
        cv2.polylines(frame, [pts], True, color, 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(frame, tag, (cx + 8, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        if len(pts) >= 2:
            side_lengths = [
                math.sqrt((pts[i][0] - pts[(i + 1) % len(pts)][0]) ** 2 +
                          (pts[i][1] - pts[(i + 1) % len(pts)][1]) ** 2)
                for i in range(len(pts))
            ]
            avg_side_px = sum(side_lengths) / len(side_lengths)
            if avg_side_px > 0:
                scale_factors.append(qr_physical_size_mm / avg_side_px)

    if len(scale_factors) < 2:
        return frame, None

    avg_scale = sum(scale_factors) / len(scale_factors)

    dx = centers["CAR"][0] - centers["WALL"][0]
    dy = centers["CAR"][1] - centers["WALL"][1]
    pixel_distance = math.sqrt(dx ** 2 + dy ** 2)
    real_world_distance_mm = pixel_distance * avg_scale - offset

    cv2.line(frame, centers["CAR"], centers["WALL"], (255, 0, 0), 2)
    mid_x = (centers["CAR"][0] + centers["WALL"][0]) // 2
    mid_y = (centers["CAR"][1] + centers["WALL"][1]) // 2
    cv2.putText(frame, f"{real_world_distance_mm:.1f} mm",
                (mid_x, mid_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    return frame, real_world_distance_mm


# ---------------------------------------------------------------------------
# Performance metrics (same logic as the simulation scripts)
# ---------------------------------------------------------------------------
def compute_metrics(t, y, setpoint, initial_value, settling_pct=0.02):
    """Compute rise time, overshoot, settling time, and steady-state error."""
    step = setpoint - initial_value
    if abs(step) < 1e-12:
        return None

    y_norm = (y - initial_value) / step

    rise_time = None
    t10 = t90 = None
    for i in range(1, len(y_norm)):
        if t10 is None and y_norm[i] >= 0.1:
            denom = y_norm[i] - y_norm[i - 1]
            if abs(denom) > 1e-15:
                frac = (0.1 - y_norm[i - 1]) / denom
                t10 = t[i - 1] + frac * (t[i] - t[i - 1])
        if t90 is None and y_norm[i] >= 0.9:
            denom = y_norm[i] - y_norm[i - 1]
            if abs(denom) > 1e-15:
                frac = (0.9 - y_norm[i - 1]) / denom
                t90 = t[i - 1] + frac * (t[i] - t[i - 1])
            break
    if t10 is not None and t90 is not None:
        rise_time = t90 - t10

    if step > 0:
        peak = np.max(y)
        peak_idx = np.argmax(y)
    else:
        peak = np.min(y)
        peak_idx = np.argmin(y)
    overshoot_abs = (peak - setpoint) if step > 0 else (setpoint - peak)
    overshoot_pct = max(overshoot_abs / abs(step) * 100, 0.0)
    t_peak = t[peak_idx]

    band = settling_pct * abs(step)
    if band < 1e-12:
        band = settling_pct * max(abs(setpoint), 1.0)
    outside = np.where(np.abs(y - setpoint) > band)[0]
    settling_time = t[outside[-1]] if len(outside) > 0 else 0.0

    n_tail = max(int(0.05 * len(y)), 1)
    ss_error = abs(np.mean(y[-n_tail:]) - setpoint)

    return {
        "rise_time": rise_time,
        "t10": t10,
        "t90": t90,
        "overshoot_pct": overshoot_pct,
        "peak": peak,
        "t_peak": t_peak,
        "settling_time": settling_time,
        "ss_error": ss_error,
    }


# ---------------------------------------------------------------------------
# Post-run plot
# ---------------------------------------------------------------------------
def show_post_run_plot(timestamps, distances, setpoint):
    """Display an annotated distance-vs-time plot after the live session."""
    if len(timestamps) < 2:
        print("Not enough data to generate a plot.")
        return

    t = np.array(timestamps)
    y = np.array(distances)
    initial_value = y[0]

    metrics = compute_metrics(t, y, setpoint, initial_value)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t, y, linewidth=2, label="Measured Distance", zorder=3)
    ax.axhline(setpoint, color="r", linestyle=":", label=f"Setpoint ({setpoint:.0f} mm)")

    if metrics is not None:
        step = setpoint - initial_value
        band = 0.02 * abs(step)
        ax.axhspan(setpoint - band, setpoint + band,
                    color="green", alpha=0.10, label="\u00b12 % band")

        if metrics["rise_time"] is not None:
            y_at_90 = initial_value + 0.9 * step
            ax.axvline(metrics["t90"], color="purple", linestyle="--",
                       linewidth=1, alpha=0.7)
            ax.plot(metrics["t90"], y_at_90, "o", color="purple",
                    markersize=7, zorder=4)
            ax.annotate(f'Rise Time = {metrics["rise_time"]:.2f} s',
                        xy=(metrics["t90"], y_at_90),
                        xytext=(metrics["t90"] + 0.3,
                                y_at_90 - 0.15 * abs(step)),
                        fontsize=9, color="purple",
                        arrowprops=dict(arrowstyle="->", color="purple",
                                        lw=1.2))

        if metrics["overshoot_pct"] > 0.5:
            ax.plot(metrics["t_peak"], metrics["peak"], "v", color="red",
                    markersize=8, zorder=4)
            ax.annotate(f'Overshoot = {metrics["overshoot_pct"]:.1f} %',
                        xy=(metrics["t_peak"], metrics["peak"]),
                        xytext=(metrics["t_peak"] + 0.4,
                                metrics["peak"] + 0.05 * abs(step)),
                        fontsize=9, color="red",
                        arrowprops=dict(arrowstyle="->", color="red",
                                        lw=1.2))
            ax.hlines(metrics["peak"], 0, metrics["t_peak"], colors="red",
                      linestyles="dotted", linewidth=0.8)

        ax.axvline(metrics["settling_time"], color="orange", linestyle="--",
                   linewidth=1, alpha=0.7)
        ax.annotate(f'Settling Time = {metrics["settling_time"]:.2f} s',
                    xy=(metrics["settling_time"], setpoint),
                    xytext=(metrics["settling_time"] + 0.3,
                            setpoint + 0.12 * abs(step)),
                    fontsize=9, color="orange",
                    arrowprops=dict(arrowstyle="->", color="orange", lw=1.2))

        rt_str = (f'{metrics["rise_time"]:.2f} s'
                  if metrics["rise_time"] is not None else "N/A")
        textstr = (f'Rise Time:   {rt_str}\n'
                   f'Overshoot:   {metrics["overshoot_pct"]:.1f} %\n'
                   f'Settling:    {metrics["settling_time"]:.2f} s\n'
                   f'SS Error:    {metrics["ss_error"]:.1f} mm')
        props = dict(boxstyle="round,pad=0.5", facecolor="wheat", alpha=0.8)
        ax.text(0.98, 0.40, textstr, transform=ax.transAxes, fontsize=9,
                verticalalignment="top", horizontalalignment="right",
                bbox=props, family="monospace")

    ax.set_title("Real-World Robot Car Response")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance to Wall (mm)")
    ax.legend(loc="lower right")
    ax.grid(True)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Main  (standalone webcam debug tool -- no robot communication)
# ---------------------------------------------------------------------------
if __name__ == "__main__":

    # --- Load camera calibration (fall back to no-distortion if missing) ---
    mtx = None
    dist_coeffs = None
    try:
        with np.load(CALIB_FILE) as data:
            mtx = data["mtx"]
            dist_coeffs = data["dist"]
        print(f"Camera calibration loaded from {CALIB_FILE}.")
    except FileNotFoundError:
        print(f"WARNING: {CALIB_FILE} not found. "
              "Running without distortion correction.")

    # --- Open webcam ---
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Could not open camera index {CAMERA_INDEX}.")
        exit()

    print(f"Setpoint distance: {SETPOINT_MM:.0f} mm")
    print("Webcam debug mode (no robot communication).")
    print("Starting video stream. Press 'q' to quit.\n")

    # --- Data log ---
    log_times = []
    log_distances = []
    t_start = time.time()
    frame_count = 0
    fps = 0.0
    fps_timer = time.time()

    grabber = CameraGrabber(cap)

    while True:
        ret, frame = grabber.retrieve()
        if not ret:
            print("Failed to grab frame.")
            break

        processed_frame, distance = process_frame_for_distance(
            frame, mtx, dist_coeffs, QR_SIZE_MM
        )

        if distance is not None:
            elapsed = time.time() - t_start
            log_times.append(elapsed)
            log_distances.append(distance)

        # --- FPS counter ---
        frame_count += 1
        now = time.time()
        if now - fps_timer >= 1.0:
            fps = frame_count / (now - fps_timer)
            frame_count = 0
            fps_timer = now

        # --- HUD overlay ---
        hud_y = 25
        cv2.putText(processed_frame, f"FPS: {fps:.1f}",
                     (10, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                     (255, 255, 255), 2)
        hud_y += 25
        cv2.putText(processed_frame,
                     f"Setpoint: {SETPOINT_MM:.0f} mm",
                     (10, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                     (255, 255, 255), 2)
        hud_y += 25
        cv2.putText(processed_frame,
                     "Debug mode (no robot)",
                     (10, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                     (200, 200, 200), 1)

        if distance is not None:
            hud_y += 25
            cv2.putText(processed_frame,
                         f"Distance: {distance:.1f} mm",
                         (10, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                         (0, 255, 255), 2)
        else:
            hud_y += 25
            cv2.putText(processed_frame,
                         "QR codes not detected",
                         (10, hud_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                         (0, 0, 255), 2)

        cv2.imshow("Live Measurement (debug)  |  Press 'q' to stop",
                   processed_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # --- Cleanup ---
    grabber.stop()
    cap.release()
    cv2.destroyAllWindows()

    print(f"\nRecorded {len(log_distances)} distance samples "
          f"over {log_times[-1]:.1f} s." if log_times else
          "\nNo distance samples recorded.")

    # --- Post-run performance plot ---
    if len(log_times) >= 2:
        show_post_run_plot(log_times, log_distances, SETPOINT_MM)
