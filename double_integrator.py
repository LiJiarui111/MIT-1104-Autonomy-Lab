import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import solve_ivp
import scipy.linalg

# ==========================================
# PID PARAMETERS (Modify these for your 5 sets)
Kp = 5.0
Ki = 0.0
Kd = 2.0
# ==========================================

# System setup
x_ref = 1.0  # Setpoint
x0 = 0.0     # Initial position
t_span = (0, 10)
t_eval = np.linspace(0, 10, 500)

# LQR Baseline Calculation
A = np.array([[0, 1], [0, 0]])
B = np.array([[0], [1]])
Q = np.diag([10, 1])
R = np.array([[1]])
P = scipy.linalg.solve_continuous_are(A, B, Q, R)
K_lqr = np.linalg.inv(R) @ (B.T @ P)

# PID Dynamics (State: [x, v, integral_error])
def pid_system(t, state):
    x, v, e_int = state
    e = x_ref - x
    de = -v
    u = Kp * e + Ki * e_int + Kd * de
    return [v, u, e]

# LQR Dynamics (State: [x, v])
def lqr_system(t, state):
    x, v = state
    u = -K_lqr[0, 0] * (x - x_ref) - K_lqr[0, 1] * v
    return [v, u]

# Simulate
sol_pid = solve_ivp(pid_system, t_span, [x0, 0.0, 0.0], t_eval=t_eval)
sol_lqr = solve_ivp(lqr_system, t_span, [x0, 0.0], t_eval=t_eval)

# ---------------------------------------------------------------------------
# Performance metrics
# ---------------------------------------------------------------------------
def compute_metrics(t, y, setpoint, initial_value, settling_pct=0.02):
    """Compute rise time, overshoot, settling time, and steady-state error."""
    step = setpoint - initial_value
    if abs(step) < 1e-12:
        return None

    y_norm = (y - initial_value) / step  # 0 at start, 1 at setpoint

    # Rise time (10% -> 90%)
    rise_time = None
    t10 = t90 = None
    for i in range(1, len(y_norm)):
        if t10 is None and y_norm[i] >= 0.1:
            frac = (0.1 - y_norm[i - 1]) / (y_norm[i] - y_norm[i - 1])
            t10 = t[i - 1] + frac * (t[i] - t[i - 1])
        if t90 is None and y_norm[i] >= 0.9:
            frac = (0.9 - y_norm[i - 1]) / (y_norm[i] - y_norm[i - 1])
            t90 = t[i - 1] + frac * (t[i] - t[i - 1])
            break
    if t10 is not None and t90 is not None:
        rise_time = t90 - t10

    # Overshoot (%)
    if step > 0:
        peak = np.max(y)
        peak_idx = np.argmax(y)
    else:
        peak = np.min(y)
        peak_idx = np.argmin(y)
    overshoot_abs = (peak - setpoint) if step > 0 else (setpoint - peak)
    overshoot_pct = max(overshoot_abs / abs(step) * 100, 0.0)
    t_peak = t[peak_idx]

    # Settling time (last time outside ±settling_pct band)
    band = settling_pct * abs(step)
    if band < 1e-12:
        band = settling_pct * max(abs(setpoint), 1.0)
    outside = np.where(np.abs(y - setpoint) > band)[0]
    settling_time = t[outside[-1]] if len(outside) > 0 else 0.0

    # Steady-state error (mean of last 5% of samples)
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

metrics = compute_metrics(sol_pid.t, sol_pid.y[0], x_ref, x0)

# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(sol_pid.t, sol_pid.y[0], label='PID Controller', linewidth=2, zorder=3)
ax.plot(sol_lqr.t, sol_lqr.y[0], '--', label='LQR (Baseline)', linewidth=2)
ax.axhline(x_ref, color='r', linestyle=':', label='Setpoint')

if metrics is not None:
    step = x_ref - x0
    band = 0.02 * abs(step)
    ax.axhspan(x_ref - band, x_ref + band, color='green', alpha=0.10, label='±2 % band')

    # Rise time marker
    if metrics["rise_time"] is not None:
        y_at_90 = x0 + 0.9 * step
        ax.axvline(metrics["t90"], color='purple', linestyle='--', linewidth=1, alpha=0.7)
        ax.plot(metrics["t90"], y_at_90, 'o', color='purple', markersize=7, zorder=4)
        ax.annotate(f'Rise Time = {metrics["rise_time"]:.2f} s',
                     xy=(metrics["t90"], y_at_90),
                     xytext=(metrics["t90"] + 0.3, y_at_90 - 0.15 * abs(step)),
                     fontsize=9, color='purple',
                     arrowprops=dict(arrowstyle='->', color='purple', lw=1.2))

    # Overshoot marker
    if metrics["overshoot_pct"] > 0.5:
        ax.plot(metrics["t_peak"], metrics["peak"], 'v', color='red', markersize=8, zorder=4)
        ax.annotate(f'Overshoot = {metrics["overshoot_pct"]:.1f} %',
                     xy=(metrics["t_peak"], metrics["peak"]),
                     xytext=(metrics["t_peak"] + 0.4, metrics["peak"] + 0.05 * abs(step)),
                     fontsize=9, color='red',
                     arrowprops=dict(arrowstyle='->', color='red', lw=1.2))
        ax.hlines(metrics["peak"], 0, metrics["t_peak"], colors='red',
                  linestyles='dotted', linewidth=0.8)

    # Settling time marker
    ax.axvline(metrics["settling_time"], color='orange', linestyle='--', linewidth=1, alpha=0.7)
    ax.annotate(f'Settling Time = {metrics["settling_time"]:.2f} s',
                 xy=(metrics["settling_time"], x_ref),
                 xytext=(metrics["settling_time"] + 0.3, x_ref + 0.12 * abs(step)),
                 fontsize=9, color='orange',
                 arrowprops=dict(arrowstyle='->', color='orange', lw=1.2))

    # Summary text box
    textstr = (f'Rise Time:   {metrics["rise_time"]:.2f} s\n'
               f'Overshoot:   {metrics["overshoot_pct"]:.1f} %\n'
               f'Settling:    {metrics["settling_time"]:.2f} s\n'
               f'SS Error:    {metrics["ss_error"]:.4f}')
    props = dict(boxstyle='round,pad=0.5', facecolor='wheat', alpha=0.8)
    ax.text(0.98, 0.40, textstr, transform=ax.transAxes, fontsize=9,
            verticalalignment='top', horizontalalignment='right',
            bbox=props, family='monospace')

ax.set_title("Double Integrator System Response")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Position (x)")
ax.legend(loc='lower right')
ax.grid(True)

# Animation
fig_anim, ax_anim = plt.subplots()
ax_anim.set_xlim(-0.2, 1.5)
ax_anim.set_ylim(-0.5, 0.5)
ax_anim.set_title("Double Integrator Animation")
ax_anim.axvline(x_ref, color='r', linestyle=':', label='Setpoint')
ax_anim.grid(True)

block_pid, = ax_anim.plot([], [], 's', markersize=20, color='blue', label='PID')
block_lqr, = ax_anim.plot([], [], 's', markersize=20, color='orange', alpha=0.6, label='LQR')
ax_anim.legend()

def init():
    block_pid.set_data([], [])
    block_lqr.set_data([], [])
    return block_pid, block_lqr

def animate(i):
    block_pid.set_data([sol_pid.y[0, i]], [0.1])
    block_lqr.set_data([sol_lqr.y[0, i]], [-0.1])
    return block_pid, block_lqr

ani = animation.FuncAnimation(fig_anim, animate, init_func=init, frames=len(t_eval), interval=20, blit=True)

plt.show()
