# Lab 7: PID Control Systems

This repository contains the necessary code and instructions for the PID Control Systems Lab (Lab 7). In this lab, you will learn about Proportional-Integral-Derivative (PID) control and how to tune the controller parameters for optimal system performance.

## Requirements

Before starting the lab, make sure you have the following installed:

1. **MATLAB R2020a or newer**
2. **Control System Toolbox** (required for the LQR ground truth implementation)

## Project Structure

```
pid_control_lab/
├── double_integrator.m        # Double integrator system simulation
├── rocket_model.m             # Rocket model (double integrator with gravity) simulation
├── pendulum.m                 # Pendulum system simulation (optional)
└── README.md                  # This file
```

## Getting Started

1. Download this repository to your local machine
2. Launch MATLAB and navigate to the lab directory
3. Read the lab handout for detailed instructions
4. Follow the lab procedure described in the handout

## Notes for Advanced Users

### Modifying Simulation Parameters

Both scripts include various simulation parameters you can modify:
- Simulation time and step size
- Initial conditions
- Reference setpoint
- System parameters (for pendulum)
- Disturbance settings

### Implementing the Pendulum Swing-Up Problem

The pendulum.py file includes a commented-out `swing_up_controller` function that implements a hybrid controller for swinging up the pendulum from the downward position. To use this:

1. Change the initial conditions to start with the pendulum pointing downward:
```python
x0 = [np.pi, 0.0, 0.0]  # Initial angle of 180 degrees (downward)
```

2. Uncomment the alternative `system` function at the bottom of the file that uses the swing-up controller.
