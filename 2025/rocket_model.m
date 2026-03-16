%% rocket_model.m - PID Control of a Rocket Model (Double Integrator with Gravity)
%
% MIT 1.104 2025

%% Clear workspace and close figures
clear;
close all;
clc;

%% PID Parameters to Tune
% Modify these values to observe their effects on system behavior
% Start by tuning K_p, then add K_i, and finally add K_d

K_p = 30.0;    % Proportional gain
K_i = 10.0;     % Integral gain
K_d = 5.0;     % Derivative gain

%% Simulation Parameters

t_start = 0.0;     % Start time (seconds)
t_end = 10.0;      % End time (seconds)
dt = 0.01;         % Time step for plotting/control (seconds)

gravity = 9.81;    % Gravitational acceleration (m/s^2)

% Initial conditions [position, velocity, error_integral]
x0 = [0.0; 0.0; 0.0];  

setpoint = 1.0;

% Add disturbance (optional)
disturbance_time = 5.0;      % Time when disturbance occurs (seconds)
disturbance_magnitude = 0.0;  % Magnitude of the impulse disturbance (set to non-zero to add disturbance)

include_lqr = true;          % Set to true to include LQR controller results

%% Run Simulation with PID Controller
% Create time points for solver and plotting
t_span = t_start:dt:t_end;

% Solve the ODE system
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, x] = ode45(@(t, x) system_dynamics(t, x, K_p, K_i, K_d, setpoint, gravity, disturbance_time, disturbance_magnitude, dt), t_span, x0, options);

% Extract the results
position = x(:, 1);
velocity = x(:, 2);
error_integral = x(:, 3);

% Calculate the control signal at each time point
control_signal = zeros(size(t));
for i = 1:length(t)
    control_signal(i) = pid_controller(t(i), x(i, :)', K_p, K_i, K_d, setpoint, disturbance_time, disturbance_magnitude, dt);
end

% Calculate error
error = setpoint - position;

%% Run LQR Controller (Ground Truth) if enabled
if include_lqr
    % Define the rocket model system in state-space form
    A = [0 1; 0 0];       % System matrix
    B = [0; 1];           % Input matrix
    
    % LQR weight matrices
    Q = diag([1, 0.1]);   % State penalty (prioritize position over velocity)
    R = 0.1;              % Control effort penalty
    
    % Compute LQR gain
    [K_lqr, ~, ~] = lqr(A, B, Q, R);
    
    % Initial conditions for LQR (position and velocity only)
    x0_lqr = [0.0; 0.0];
    
    % Simulate LQR system
    [t_lqr, x_lqr] = ode45(@(t, x) lqr_system(t, x, A, B, K_lqr, setpoint, gravity), t_span, x0_lqr, options);
    
    % Extract results
    position_lqr = x_lqr(:, 1);
    velocity_lqr = x_lqr(:, 2);
    
    % Calculate control signal
    control_signal_lqr = zeros(size(t_lqr));
    for i = 1:length(t_lqr)
        % LQR control law with gravity compensation and setpoint tracking
        control_signal_lqr(i) = -K_lqr * ([position_lqr(i); velocity_lqr(i)] - [setpoint; 0]) + gravity;
    end
    
    % Calculate error
    error_lqr = setpoint - position_lqr;
    
    % Calculate LQR performance metrics
    % Rise time
    rise_time_indices_lqr = find(position_lqr >= 0.9*setpoint);
    if ~isempty(rise_time_indices_lqr)
        rise_time_lqr = t_lqr(rise_time_indices_lqr(1));
    else
        rise_time_lqr = Inf;
    end
    
    % Maximum overshoot
    max_overshoot_lqr = max(0, max(position_lqr) - setpoint) / setpoint * 100;
    
    % Settling time
    settling_threshold = 0.02 * setpoint;
    settling_time_lqr = Inf;
    for i = 1:length(t_lqr)
        if i >= rise_time_indices_lqr(1) && all(abs(position_lqr(i:end) - setpoint) <= settling_threshold)
            settling_time_lqr = t_lqr(i);
            break;
        end
    end
    
    % Steady-state error
    steady_state_error_lqr = abs(setpoint - position_lqr(end));
end

%% Calculate Performance Metrics for PID
% Rise time (time to reach 90% of setpoint)
rise_time_indices = find(position >= 0.9*setpoint);
if ~isempty(rise_time_indices)
    rise_time = t(rise_time_indices(1));
else
    rise_time_indices(1) = Inf; 
    rise_time = Inf;
end

% Maximum overshoot
max_overshoot = max(0, max(position) - setpoint) / setpoint * 100;

% Settling time (time to stay within 2% of setpoint)
settling_threshold = 0.02 * setpoint;
settling_time = Inf;
for i = 1:length(t)
    if i >= rise_time_indices(1) && all(abs(position(i:end) - setpoint) <= settling_threshold)
        settling_time = t(i);
        break;
    end
end

% Steady-state error
steady_state_error = abs(setpoint - position(end));

%% Plot Results
figure('Name', 'Rocket Model PID Control', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);

% Plot position vs time
subplot(3, 1, 1);
plot(t, position, 'b-', 'LineWidth', 2);
hold on;
plot(t, setpoint*ones(size(t)), 'r--', 'LineWidth', 1);
if include_lqr
    plot(t_lqr, position_lqr, 'g-', 'LineWidth', 1.5);
end
if disturbance_magnitude ~= 0
    xline(disturbance_time, 'k--', 'Disturbance');
end
xlabel('Time (s)');
ylabel('Position');
grid on;
if include_lqr
    legend('PID Control', 'Setpoint', 'Ground Truth (LQR)');
else
    legend('Position', 'Setpoint');
end
title('System Response');

% Add annotations for performance metrics
if ~isinf(rise_time)
    text(rise_time, 0.9*setpoint, ['\leftarrow Rise Time = ', num2str(rise_time, '%.2f'), 's']);
end

if max_overshoot > 0
    [max_pos, max_pos_idx] = max(position);
    text(t(max_pos_idx), max_pos, ['\leftarrow Overshoot = ', num2str(max_overshoot, '%.2f'), '%']);
end

if ~isinf(settling_time)
    text(settling_time, setpoint, ['\leftarrow Settling Time = ', num2str(settling_time, '%.2f'), 's']);
end

% Plot control signal vs time
subplot(3, 1, 2);
plot(t, control_signal, 'b-', 'LineWidth', 2);
hold on;
yline(gravity, 'r--', ['Gravity (', num2str(gravity), ' N)']);
if include_lqr
    plot(t_lqr, control_signal_lqr, 'g-', 'LineWidth', 1.5);
    legend('PID Control', 'Gravity Force', 'Ground Truth (LQR)');
else
    legend('Control Signal', 'Gravity Force');
end
xlabel('Time (s)');
ylabel('Control Signal (Force)');
grid on;
title('Control Signal');

% Plot error vs time
subplot(3, 1, 3);
plot(t, error, 'b-', 'LineWidth', 2);
hold on;
if include_lqr
    plot(t_lqr, error_lqr, 'g-', 'LineWidth', 1.5);
    legend('PID Control', 'Ground Truth (LQR)');
end
xlabel('Time (s)');
ylabel('Error');
grid on;
title('Error');

sgtitle(['Rocket Model PID Control (K_p=', num2str(K_p), ', K_i=', num2str(K_i), ', K_d=', num2str(K_d), ')'], 'FontSize', 14);

% Print performance metrics
fprintf('\nPID Performance Metrics:\n');
fprintf('Rise Time: %.2f s\n', rise_time);
fprintf('Maximum Overshoot: %.2f%%\n', max_overshoot);
fprintf('Settling Time: %.2f s\n', settling_time);
fprintf('Steady State Error: %.6f\n', steady_state_error);

% Print control signal analysis
final_control = control_signal(end);
fprintf('\nFinal Control Signal: %.2f N\n', final_control);
fprintf('Gravity Force: %.2f N\n', gravity);
fprintf('Difference: %.2f N\n', final_control - gravity);

if include_lqr
    fprintf('\nGround Truth (LQR) Performance Metrics:\n');
    fprintf('Rise Time: %.2f s\n', rise_time_lqr);
    fprintf('Maximum Overshoot: %.2f%%\n', max_overshoot_lqr);
    fprintf('Settling Time: %.2f s\n', settling_time_lqr);
    fprintf('Steady State Error: %.6f\n', steady_state_error_lqr);
    
    final_control_lqr = control_signal_lqr(end);
    fprintf('\nFinal LQR Control Signal: %.2f N\n', final_control_lqr);
    fprintf('Gravity Force: %.2f N\n', gravity);
    fprintf('Difference: %.2f N\n', final_control_lqr - gravity);
end

%% System Dynamics Functions
function dxdt = system_dynamics(t, x, K_p, K_i, K_d, setpoint, gravity, disturbance_time, disturbance_magnitude, dt)
    % Calculate control input
    u = pid_controller(t, x, K_p, K_i, K_d, setpoint, disturbance_time, disturbance_magnitude, dt);
    
    % Rocket model dynamics: ẍ = u - gravity
    position = x(1);
    velocity = x(2);
    
    % Calculate error for integral term
    error = setpoint - position;
    
    % State derivatives
    dxdt = [velocity; u - gravity; error];
end

function u = pid_controller(t, x, K_p, K_i, K_d, setpoint, disturbance_time, disturbance_magnitude, dt)
    position = x(1);
    velocity = x(2);
    error_integral = x(3);
    
    % Calculate error
    error = setpoint - position;
    
    % PID control components
    p_term = K_p * error;                % Proportional term
    i_term = K_i * error_integral;       % Integral term
    d_term = -K_d * velocity;            % Derivative term (note: d/dt(error) = -velocity for constant setpoint)
    
    % Total control signal
    u = p_term + i_term + d_term;
    
    % Add disturbance at specific time if enabled
    if abs(t - disturbance_time) < dt && disturbance_magnitude ~= 0
        u = u + disturbance_magnitude;
    end
end

function dxdt = lqr_system(t, x, A, B, K, setpoint, gravity)
    % LQR closed-loop system dynamics with gravity and setpoint tracking
    
    % Calculate control input (with gravity compensation)
    u = -K * (x - [setpoint; 0]) + gravity;
    
    % System dynamics
    dxdt = A * x + B * (u - gravity);
end