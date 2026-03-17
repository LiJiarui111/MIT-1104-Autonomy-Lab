# Lab 7: PID Control Systems

This repository contains the simulation scripts, real-world measurement code, and instructions for **Lab 7: Autonomy** (MIT 1.104). In this lab you will learn how Proportional-Integral-Derivative (PID) controllers work by tuning them on simulated systems and then testing on a real robot car.

## Project Structure

```
MIT-1104-Lab7-PID-2025/
├── simulation/
│   ├── double_integrator.py   # Part 1  – Double integrator system
│   ├── rocket_model.py        # Part 2  – Rocket model (double integrator + gravity)
│   └── pendulum.py            # Part 3  – Nonlinear pendulum (optional)
├── real_world/
│   ├── run_experiment.py      # Part 1b – Student-facing real-world experiment script
│   ├── live_measurement.py    # Webcam QR-code distance utilities (also standalone debug tool)
│   └── robot_car/
│       └── robot_car.ino      # ESP32 Arduino firmware (uploaded once by TA)
├── 2026.tex                   # Lab handout (LaTeX source)
├── requirements.txt           # Python dependencies
└── README.md                  # This file
```

## Prerequisites

- **Python 3.8 or newer** (check with `python --version`)
- A terminal / command prompt (the built-in VS Code terminal works well)
- For the real-world test: a USB webcam and Bluetooth pairing with the robot car

## Setup

1. Clone the repository:

   ```bash
   git clone https://github.com/LiJiarui111/MIT-1104-Autonomy-Lab
   cd MIT-1104-Autonomy-Lab
   ```

2. Install the required packages:

   ```bash
   pip install -r requirements.txt
   ```

## Running the Simulation Scripts

Each simulation script models a different system. Open the file in VS Code, set the PID gains (`Kp`, `Ki`, `Kd`) at the top, save, then run:

```bash
python simulation/double_integrator.py    # Part 1
python simulation/rocket_model.py         # Part 2
python simulation/pendulum.py             # Part 3 (optional)
```

Two windows will appear for each script:
- A **line plot** showing the system response over time (PID vs. LQR baseline), with performance metrics annotated directly on the plot.
- An **animation** showing the system in motion.

## Running the Real-World Test

The real-world experiment sends PID parameters to the robot car from the laptop via Bluetooth -- no re-flashing needed. The student-facing script is `real_world/run_experiment.py`.

1. Open `real_world/run_experiment.py` and set the PID gains and serial port at the top:

   ```python
   Kp = 5.0
   Ki = 0.0
   Kd = 2.0
   SETPOINT_MM  = 300.0        # Target distance from wall (mm)
   SERIAL_PORT  = "COM5"       # Bluetooth serial port (see label on car)
                                # macOS: "/dev/tty.RobotCar_XX"
   CAMERA_INDEX = 0             # Webcam index (0 = built-in, 1+ = USB)
   ```

2. Make sure your laptop is paired with the robot car's Bluetooth (the TA should have done this already; the Bluetooth name and serial port are on the car's label).

3. Run the script:

   ```bash
   python real_world/run_experiment.py
   ```

   The script connects via Bluetooth serial, sends PID gains and a START command to the car, then streams webcam distance measurements in real time.

4. Press `q` to stop. The script sends a STOP command to the car and displays a post-run performance plot with annotated metrics.

5. To try different gains, edit the file and re-run -- no need to touch the robot car.

> **`live_measurement.py`** is a standalone debug tool for testing the webcam and QR code detection without communicating with the robot. Run it with `python real_world/live_measurement.py`.

### QR Code Labels

Print two QR codes at the same physical size (matching `QR_SIZE_MM`):
- One encoding the text **`CAR`** (attach to the robot car)
- One encoding the text **`WALL`** (attach to the wall)

## Flashing the Robot Car (TA only -- done once before the lab)

The robot car runs a SparkFun ESP32 RedBoard with the Arduino sketch `real_world/robot_car/robot_car.ino`. The firmware is **uploaded once** by the TA; PID gains are **not** set in the sketch -- they are sent from the laptop at runtime via Bluetooth.

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) and add the **ESP32 board package** (Board Manager URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`).
2. Open `real_world/robot_car/robot_car.ino` in the Arduino IDE.
3. Set the Bluetooth device name (`BT_NAME`) at the top of the file. Each car should have a unique name (e.g. `RobotCar_01`).
4. Select board **SparkFun ESP32 RedBoard** (or **ESP32 Dev Module**), choose the correct COM port, and click **Upload**.
5. Open the Serial Monitor at **115200 baud**. You should see `Bluetooth: RobotCar_XX` and `STATE   IDLE`.

The car boots into **IDLE** mode (motors off, Bluetooth active). It waits for a `PID:...` configuration message and a `START` command from the laptop before activating the control loop. **No re-flashing is needed between student runs.**

### Pin Wiring Reference

| Component | ESP32 Pin |
|---|---|
| L298N ENA (Motor A PWM) | GPIO 25 |
| L298N IN1, IN2 | GPIO 26, 27 |
| L298N ENB (Motor B PWM) | GPIO 14 |
| L298N IN3, IN4 | GPIO 12, 13 |
| HC-SR04 TRIG | GPIO 5 |
| HC-SR04 ECHO | GPIO 18 |

## Modifying PID Gains

At the top of every simulation script you will find a clearly marked block:

```python
# ==========================================
# PID PARAMETERS (Modify these for your 5 sets)
Kp = 5.0
Ki = 0.0
Kd = 2.0
# ==========================================
```

Change the values, save the file, and re-run the script to see the effect.

## Performance Metrics

Both simulation and real-world plots automatically compute and display:

| Metric | Definition |
|---|---|
| **Rise Time** | Time for the response to go from 10 % to 90 % of the setpoint change |
| **Overshoot** | Maximum amount the response exceeds the setpoint (as a %) |
| **Settling Time** | Time after which the response stays within +/-2 % of the setpoint |
| **Steady-State Error** | Difference between the final value and the setpoint |

Use these values to fill in the tables in your lab report.

## Troubleshooting

- **`python` not found**: On Windows, make sure Python is added to PATH (check "Add Python to PATH" during install). On macOS/Linux, try `python3` instead.
- **`pyzbar` import error on Windows**: Install the [Microsoft Visual C++ Redistributable](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist) (usually pre-installed). No `brew install zbar` needed on Windows -- `pip install pyzbar` bundles everything.
- **Bluetooth serial port not found**: On Windows, open **Device Manager > Ports (COM & LPT)** and look for "Standard Serial over Bluetooth link". Use the **outgoing** COM port number (e.g. `COM5`). On macOS, run `ls /dev/tty.Robot*`.
- **Packages fail to install**: Make sure `pip` is up to date (`pip install --upgrade pip`).
- **Wrong camera opens**: Change `CAMERA_INDEX` to `1` or `2` in `run_experiment.py` (or `live_measurement.py` if debugging).
- **Camera calibration file not found**: The script will run without distortion correction and print a warning. For best accuracy, provide a `camera_calib.npz` file.
- **Matplotlib window does not appear**: On macOS you may need `pip install pyobjc`.

## References

See the lab handout (`2026.tex`) for the full theoretical background and detailed procedure.
