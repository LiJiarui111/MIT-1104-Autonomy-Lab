#!/usr/bin/env python3
"""
MIT 1.104 Lab 7 – Real-World Robot Car Experiment

Run this script to execute a PID control trial on the physical robot car.
Edit the PID parameters below, then run:

    python real_world/run_experiment.py

The script will:
  1. Connect to the robot car via Bluetooth serial.
  2. Send the PID parameters and a START command to the car.
  3. Stream webcam distance measurements to the car in real time.
  4. When you press 'q', send STOP and display a performance plot.
"""

import cv2
import numpy as np
import serial
import time
import sys
import threading
import queue

from live_measurement import (
    CameraGrabber,
    process_frame_for_distance,
    show_post_run_plot,
)

# ==========================================
# PID PARAMETERS  (Same as in double_integrator.py)
# ==========================================
Kp = 5.0
Ki = 0.0
Kd = 2.0
# ==========================================

# ==========================================
# EXPERIMENT CONFIGURATION
# ==========================================
SETPOINT_MM  = 100.0               # Target distance from wall (mm)
SERIAL_PORT  = "COM8"              # Bluetooth serial port (see label on car)
                                   # Windows: "COM5" (check Device Manager > Ports)
                                   # macOS:   "/dev/tty.RobotCar_XX"
SERIAL_BAUD  = 115200              # Baud rate (must match ESP32)
CAMERA_INDEX = 1                   # Webcam index (0 = built-in, 1+ = USB)
QR_SIZE_MM   = 75.0                # Physical side length of each QR code (mm)
CALIB_FILE   = "camera_calib.npz"  # Camera calibration file (optional)
CONTROL_HZ   = 20                 # Must match ESP32 CONTROL_HZ

OFFSET = 260 # mm
# ==========================================


def main():
    # --- Load camera calibration (fall back to no-distortion if missing) ---
    mtx = None
    dist_coeffs = None
    try:
        with np.load(CALIB_FILE) as data:
            mtx = data["mtx"]
            dist_coeffs = data["dist"]
        print(f"Camera calibration loaded from {CALIB_FILE}.")
    except FileNotFoundError:
        print(f"Note: {CALIB_FILE} not found. Running without distortion correction.")

    # --- Open webcam ---
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: Could not open camera index {CAMERA_INDEX}.")
        return

    # --- Open Bluetooth serial connection ---
    print(f"\nConnecting to robot car on {SERIAL_PORT} ...")
    try:
        bt_serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1,
                                   write_timeout=0.01)
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}: {e}")
        print("Make sure the robot car is powered on and paired via Bluetooth.")
        print("  Windows: check Device Manager > Ports for the correct COM number.")
        print("  macOS:   run  ls /dev/tty.Robot*  in a terminal.")
        cap.release()
        return
    time.sleep(0.5)
    print("Connected.")

    tx_queue = queue.Queue(maxsize=32)
    tx_stop = threading.Event()
    tx_stats = {"sent": 0, "dropped": 0, "errors": 0}

    def send_blocking(msg: str):
        try:
            bt_serial.write((msg + "\n").encode("utf-8"))
        except serial.SerialException as e:
            print(f"  [BT send error: {e}]")

    def send(msg: str):
        try:
            tx_queue.put_nowait(msg)
            return
        except queue.Full:
            pass
        # Drop oldest to keep newest control value flowing.
        try:
            tx_queue.get_nowait()
            tx_queue.task_done()
        except queue.Empty:
            pass
        tx_stats["dropped"] += 1
        try:
            tx_queue.put_nowait(msg)
        except queue.Full:
            tx_stats["dropped"] += 1

    def serial_writer():
        while not tx_stop.is_set() or not tx_queue.empty():
            try:
                msg = tx_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            try:
                bt_serial.write((msg + "\n").encode("utf-8"))
                tx_stats["sent"] += 1
            except serial.SerialException as e:
                tx_stats["errors"] += 1
                print(f"  [BT send error: {e}]")
            finally:
                tx_queue.task_done()

    tx_thread = threading.Thread(target=serial_writer, daemon=True)
    tx_thread.start()

    # --- Send PID configuration ---
    pid_msg = f"PID:{Kp},{Ki},{Kd},{SETPOINT_MM}"
    print(f"Sending PID config:  {pid_msg}")
    send_blocking(pid_msg)
    time.sleep(0.1)

    # --- Send START command ---
    print("Sending START command.")
    send_blocking("START")
    time.sleep(0.05)

    print(f"Streaming webcam distance via {SERIAL_PORT}")
    print(f"Setpoint: {SETPOINT_MM:.0f} mm  |  Kp={Kp}  Ki={Ki}  Kd={Kd}")
    print("Press 'q' in the video window to stop.\n")

    # --- Data log ---
    log_times = []
    log_distances = []
    t_start = time.time()
    frame_count = 0
    fps = 0.0
    fps_timer = time.time()

    send_interval = 1.0 / CONTROL_HZ
    last_send_time = 0.0
    late_send_count = 0
    last_cam_warn_ts = 0.0

    grabber = CameraGrabber(cap)

    try:
        while True:
            ret, frame = grabber.read()
            if not ret:
                now_warn = time.time()
                if now_warn - last_cam_warn_ts >= 1.0:
                    age = grabber.frame_age_s()
                    age_str = f"{age:.2f}s" if np.isfinite(age) else "inf"
                    print(f"[Camera timeout/stale] last frame age={age_str}")
                    last_cam_warn_ts = now_warn
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
                continue

            processed_frame, distance = process_frame_for_distance(
                frame, mtx, dist_coeffs, QR_SIZE_MM, OFFSET
            )

            if distance is not None:
                elapsed = time.time() - t_start
                log_times.append(elapsed)
                log_distances.append(distance)

                now_send = time.time()
                if now_send - last_send_time >= send_interval:
                    if last_send_time > 0 and (now_send - last_send_time) > (1.5 * send_interval):
                        late_send_count += 1
                    send(f"DIST:{distance:.1f}")
                    last_send_time = now_send

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
                        f"Kp={Kp}  Ki={Ki}  Kd={Kd}",
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

            cv2.imshow("Robot Car Experiment  |  Press 'q' to stop",
                       processed_frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    # --- Send STOP and clean up ---
    print("\nSending STOP command.")
    tx_stop.set()
    tx_queue.join()
    send_blocking("STOP")
    time.sleep(0.1)
    tx_thread.join(timeout=1.0)

    grabber.stop()
    cap.release()
    cv2.destroyAllWindows()
    bt_serial.close()

    if log_times:
        print(f"Recorded {len(log_distances)} distance samples "
              f"over {log_times[-1]:.1f} s.")
    else:
        print("No distance samples recorded.")
    print(f"TX stats: sent={tx_stats['sent']} dropped={tx_stats['dropped']} "
          f"errors={tx_stats['errors']} late_ticks={late_send_count}")

    # --- Post-run performance plot ---
    if len(log_times) >= 2:
        show_post_run_plot(log_times, log_distances, SETPOINT_MM)


if __name__ == "__main__":
    main()
