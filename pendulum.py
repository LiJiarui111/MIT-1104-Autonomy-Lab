import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.integrate import solve_ivp
import scipy.linalg

# ==========================================
# PID PARAMETERS (Modify these for your 5 sets)
Kp = 20.0
Ki = 5.0
Kd = 5.0
# ==========================================

# Pendulum parameters
m, L, g_const, b = 1.0, 1.0, 9.81, 0.1
theta_ref = 0.0  # 0 is perfectly upright
theta_initial = np.pi / 4  # Starting slightly tipped over

t_span = (0, 10)
t_eval = np.linspace(0, 10, 500)

# LQR Baseline (Linearized at theta=0)
A = np.array([[0, 1], [g_const/L, -b/(m*L**2)]])
B = np.array([[0], [1/(m*L**2)]])
Q = np.diag([10, 1])
R = np.array([[1]])
P = scipy.linalg.solve_continuous_are(A, B, Q, R)
K_lqr = np.linalg.inv(R) @ (B.T @ P)

# PID Dynamics (Nonlinear system)
def pid_system(t, state):
    theta, omega, e_int = state
    e = theta_ref - theta
    de = -omega
    u = Kp * e + Ki * e_int + Kd * de
    alpha = (g_const/L)*np.sin(theta) - (b/(m*L**2))*omega + u/(m*L**2)
    return [omega, alpha, e]

# LQR Dynamics (Nonlinear system with linear LQR control)
def lqr_system(t, state):
    theta, omega = state
    u = -K_lqr[0, 0] * (theta - theta_ref) - K_lqr[0, 1] * omega
    alpha = (g_const/L)*np.sin(theta) - (b/(m*L**2))*omega + u/(m*L**2)
    return [omega, alpha]

sol_pid = solve_ivp(pid_system, t_span, [theta_initial, 0.0, 0.0], t_eval=t_eval)
sol_lqr = solve_ivp(lqr_system, t_span, [theta_initial, 0.0], t_eval=t_eval)

# ---------------------------------------------------------------------------
# Performance metrics (handles response approaching setpoint from above)
# ---------------------------------------------------------------------------
def compute_metrics(t, y, setpoint, initial_value, settling_pct=0.02):
    """Compute rise time, overshoot, settling time, and steady-state error.

    Works for both step-up (initial < setpoint) and step-down (initial > setpoint).
    """
    step = setpoint - initial_value
    if abs(step) < 1e-12:
        return None

    y_norm = (y - initial_value) / step  # 0 at start, 1 at setpoint

    # Rise time (10% -> 90% of normalised response)
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

    # Overshoot (%)
    # "Overshoot" is how far past the setpoint the response goes.
    if step > 0:
        peak = np.max(y)
        peak_idx = np.argmax(y)
        overshoot_abs = peak - setpoint
    else:
        peak = np.min(y)
        peak_idx = np.argmin(y)
        overshoot_abs = setpoint - peak
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

metrics = compute_metrics(sol_pid.t, sol_pid.y[0], theta_ref, theta_initial)

# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(sol_pid.t, sol_pid.y[0], label='PID Controller', linewidth=2, zorder=3)
ax.plot(sol_lqr.t, sol_lqr.y[0], '--', label='LQR (Baseline)', linewidth=2)
ax.axhline(theta_ref, color='r', linestyle=':', label='Upright Setpoint (0 rad)')

if metrics is not None:
    step = theta_ref - theta_initial  # negative for pendulum
    band = 0.02 * abs(step)
    ax.axhspan(theta_ref - band, theta_ref + band, color='green', alpha=0.10, label='±2 % band')

    # Rise time marker
    if metrics["rise_time"] is not None:
        y_at_90 = theta_initial + 0.9 * step
        ax.axvline(metrics["t90"], color='purple', linestyle='--', linewidth=1, alpha=0.7)
        ax.plot(metrics["t90"], y_at_90, 'o', color='purple', markersize=7, zorder=4)
        ax.annotate(f'Rise Time = {metrics["rise_time"]:.2f} s',
                     xy=(metrics["t90"], y_at_90),
                     xytext=(metrics["t90"] + 0.3, y_at_90 + 0.10 * abs(step)),
                     fontsize=9, color='purple',
                     arrowprops=dict(arrowstyle='->', color='purple', lw=1.2))

    # Overshoot marker
    if metrics["overshoot_pct"] > 0.5:
        ax.plot(metrics["t_peak"], metrics["peak"], 'v', color='red', markersize=8, zorder=4)
        yt_off = -0.08 * abs(step) if step < 0 else 0.08 * abs(step)
        ax.annotate(f'Overshoot = {metrics["overshoot_pct"]:.1f} %',
                     xy=(metrics["t_peak"], metrics["peak"]),
                     xytext=(metrics["t_peak"] + 0.4, metrics["peak"] + yt_off),
                     fontsize=9, color='red',
                     arrowprops=dict(arrowstyle='->', color='red', lw=1.2))
        ax.hlines(metrics["peak"], 0, metrics["t_peak"], colors='red',
                  linestyles='dotted', linewidth=0.8)

    # Settling time marker
    ax.axvline(metrics["settling_time"], color='orange', linestyle='--', linewidth=1, alpha=0.7)
    ax.annotate(f'Settling Time = {metrics["settling_time"]:.2f} s',
                 xy=(metrics["settling_time"], theta_ref),
                 xytext=(metrics["settling_time"] + 0.3, theta_ref + 0.15 * abs(step)),
                 fontsize=9, color='orange',
                 arrowprops=dict(arrowstyle='->', color='orange', lw=1.2))

    # Summary text box
    rt_str = f'{metrics["rise_time"]:.2f} s' if metrics["rise_time"] is not None else 'N/A'
    textstr = (f'Rise Time:   {rt_str}\n'
               f'Overshoot:   {metrics["overshoot_pct"]:.1f} %\n'
               f'Settling:    {metrics["settling_time"]:.2f} s\n'
               f'SS Error:    {metrics["ss_error"]:.4f}')
    props = dict(boxstyle='round,pad=0.5', facecolor='wheat', alpha=0.8)
    ax.text(0.98, 0.97, textstr, transform=ax.transAxes, fontsize=9,
            verticalalignment='top', horizontalalignment='right',
            bbox=props, family='monospace')

ax.set_title("Pendulum System Response")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Angle (radians)")
ax.legend(loc='upper right', bbox_to_anchor=(0.98, 0.72))
ax.grid(True)

# Animation
fig_anim, ax_anim = plt.subplots()
ax_anim.set_xlim(-1.5, 1.5)
ax_anim.set_ylim(-1.5, 1.5)
ax_anim.set_aspect('equal')
ax_anim.set_title("Pendulum Animation (0 is Upright)")
ax_anim.grid(True)

line_pid, = ax_anim.plot([], [], 'o-', lw=3, markersize=10, color='blue', label='PID')
line_lqr, = ax_anim.plot([], [], 'o-', lw=3, markersize=10, color='orange', alpha=0.6, label='LQR')
ax_anim.legend()

def init():
    line_pid.set_data([], [])
    line_lqr.set_data([], [])
    return line_pid, line_lqr

def animate(i):
    x_pid = L * np.sin(sol_pid.y[0, i])
    y_pid = L * np.cos(sol_pid.y[0, i])
    x_lqr = L * np.sin(sol_lqr.y[0, i])
    y_lqr = L * np.cos(sol_lqr.y[0, i])

    line_pid.set_data([0, x_pid], [0, y_pid])
    line_lqr.set_data([0, x_lqr], [0, y_lqr])
    return line_pid, line_lqr

ani = animation.FuncAnimation(fig_anim, animate, init_func=init, frames=len(t_eval), interval=20, blit=True)

plt.show()
