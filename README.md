# Lab 7: PID Control Systems

This repository contains the simulation scripts and instructions for **Lab 7: Autonomy** (MIT 1.104). In this lab you will learn how Proportional-Integral-Derivative (PID) controllers work by tuning them on three progressively harder systems.

## Project Structure

```
MIT-1104-Lab7-PID-2025/
├── double_integrator.py   # Part 1 – Double integrator system
├── rocket_model.py        # Part 2 – Rocket model (double integrator + gravity)
├── pendulum.py            # Part 3 – Nonlinear pendulum (extra credit)
├── instruction.tex        # Lab handout (LaTeX source)
├── requirements.txt       # Python dependencies
└── README.md              # This file
```

## Prerequisites

- **Python 3.8 or newer** (check with `python --version`)
- A terminal / command prompt (the built-in VS Code terminal works well)

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

## Running the Scripts

Each script simulates a different system. Open the file in VS Code, set the PID gains (`Kp`, `Ki`, `Kd`) at the top, save, then run:

```bash
python double_integrator.py    # Part 1
python rocket_model.py         # Part 2
python pendulum.py             # Part 3 (extra credit)
```

Two windows will appear for each script:
- A **line plot** showing the system response over time (PID vs. LQR baseline), with performance metrics annotated directly on the plot.
- An **animation** showing the system in motion.

## Modifying PID Gains

At the top of every script you will find a clearly marked block:

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

Each plot automatically computes and displays:

| Metric | Definition |
|---|---|
| **Rise Time** | Time for the response to go from 10 % to 90 % of the setpoint change |
| **Overshoot** | Maximum amount the response exceeds the setpoint (as a %) |
| **Settling Time** | Time after which the response stays within ±2 % of the setpoint |
| **Steady-State Error** | Difference between the final value and the setpoint |

Use these values to fill in the tables in your lab report.

## Troubleshooting

- **`python` not found**: Try `python3` instead.
- **Matplotlib window does not appear**: Make sure you are not running inside a headless SSH session. On macOS you may need to install the `pyobjc` framework (`pip install pyobjc`).
- **Packages fail to install**: Make sure `pip` is up to date (`pip install --upgrade pip`).

## References

See the lab handout for the full theoretical background and detailed procedure.
