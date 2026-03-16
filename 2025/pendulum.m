%% pendulum.m - PID Control of a Pendulum System
%
% MIT 1.104 Lab 7 2025

%% Clear workspace and close figures
clear;
close all;
clc;

%% PID Parameters to Tune
% Modify these values to observe their effects on system behavior
% Start by tuning K_p, then add K_i, and finally add K_d

K_p = 50.0;    % Proportional gain
K_i = 2.0;     % Integral gain
K_d = 2.0;    % Derivative gain

%% Simulation Parameters

t_start = 0.0;     % Start time (seconds)
t_end = 10.0;      % End time (seconds)
dt = 0.01;         % Time step for plotting/control (seconds)

% Pendulum parameters
m = 1.0;      % Mass (kg)
l = 1.0;      % Length (m)
g = 9.81;     % Gravity (m/s^2)
b = 0.5;      % Damping coefficient (kg*m^2/s)

% Initial conditions [angle, angular velocity, error_integral]
x0 = [pi/4; 0.0; 0.0];  % Initial angle close to downward position

setpoint = 0.0;     % Balance the pendulum in the upright position

% Add disturbance (optional)
disturbance_time = 5.0;      % Time when disturbance occurs (seconds)
disturbance_magnitude = 0.0;  % Magnitude of the impulse disturbance (set to non-zero to add disturbance)

% Animation parameters
create_animation = true;    % Set to true to create an animation of the pendulum

include_lqr = true;          % Set to true to include LQR controller results

%% Define System Dynamics Functions
% Pendulum system dynamics for PID controller
function dxdt = pendulum_dynamics_pid(t, x, K_p, K_i, K_d, setpoint, m, l, g, b, disturbance_time, disturbance_magnitude, dt)
    u = pid_controller(t, x, K_p, K_i, K_d, setpoint, disturbance_time, disturbance_magnitude, dt);

    theta = x(1);
    omega = x(2);
    
    angular_acceleration = (m*g*l*sin(theta) - b*omega + u) / (m*l^2);
    
    error = setpoint - theta;
    
    dxdt = [omega; angular_acceleration; error];
end

% PID controller function
function u = pid_controller(t, x, K_p, K_i, K_d, setpoint, disturbance_time, disturbance_magnitude, dt)
    theta = x(1);
    omega = x(2);
    error_integral = x(3);
    
    error = setpoint - theta;
    
    % PID control components
    p_term = K_p * error;                % Proportional term
    i_term = K_i * error_integral;       % Integral term
    d_term = -K_d * omega;               % Derivative term (note: d/dt(error) = -omega for constant setpoint)
    
    u = p_term + i_term + d_term;
    
    if abs(t - disturbance_time) < dt && disturbance_magnitude ~= 0
        u = u + disturbance_magnitude;
    end
    % u = 0; 
end

% LQR pendulum system dynamics
function dxdt = pendulum_dynamics_lqr(t, x, K, m, l, g, b, setpoint)
    theta = x(1);
    omega = x(2);
    
    u = -K * (x - [setpoint; 0]);
    
    angular_acceleration = (m*g*l*sin(theta) - b*omega + u) / (m*l^2);
    
    dxdt = [omega; angular_acceleration];
end

%% Animation function
function animate_pendulum(t, theta, control_signal, l, include_lqr, t_lqr, theta_lqr, control_signal_lqr)
    % Creates an animation of the pendulum motion
    
    % Setup figure
    figure('Name', 'Pendulum Animation', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);
    
    % Set axis limits
    axis_limit = 1.5*l;
    ax = gca;
    ax.XLim = [-axis_limit, axis_limit];
    ax.YLim = [-axis_limit, axis_limit];
    axis equal;
    grid on;
    hold on;
    
    % Create visualization elements for PID controller
    pivot = plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');  % Pivot point
    rod = plot([0, 0], [0, 0], 'b-', 'LineWidth', 2);  % Pendulum rod
    bob = plot(0, 0, 'bo', 'MarkerSize', 20, 'MarkerFaceColor', 'b');  % Pendulum bob
    arrow = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);  % Control force arrow
    
    % Create visualization elements for LQR if included
    if include_lqr
        rod_lqr = plot([0, 0], [0, 0], 'g-', 'LineWidth', 1.5);  % LQR pendulum rod
        bob_lqr = plot(0, 0, 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g');  % LQR pendulum bob
    end
    
    % Draw upright position reference
    plot([0, 0], [0, l], 'r--', 'LineWidth', 1);
    
    % Create legend
    if include_lqr
        legend('Pivot', 'PID Pendulum', 'PID Bob', 'Control Force', 'LQR Pendulum', 'LQR Bob', 'Upright Position', 'Location', 'best');
    else
        legend('Pivot', 'Pendulum Rod', 'Pendulum Bob', 'Control Force', 'Upright Position', 'Location', 'best');
    end
    
    % Text for time and control signal display
    time_text = text(-axis_limit*0.9, axis_limit*0.9, '', 'FontSize', 12);
    control_text = text(-axis_limit*0.9, axis_limit*0.8, '', 'FontSize', 12);
    
    title('Pendulum Swing-Up with PID Control');
    
    % Interpolate LQR data to match PID time points if needed
    if include_lqr && length(t) ~= length(t_lqr)
        theta_lqr_interp = interp1(t_lqr, theta_lqr, t);
    else
        theta_lqr_interp = theta_lqr;
    end
    
    % Animation loop
    for i = 1:5:length(t)  % Skip frames to speed up animation
        % Calculate pendulum position for PID
        x_pos = l * sin(theta(i));
        y_pos = l * cos(theta(i));
        
        % Update rod
        set(rod, 'XData', [0, x_pos], 'YData', [0, y_pos]);
        
        % Update bob
        set(bob, 'XData', x_pos, 'YData', y_pos);
        
        % Draw control force as an arrow
        % Scale control signal for visualization
        scale = 0.1;
        control = control_signal(i) * scale;
        
        % Direction perpendicular to the rod
        arrow_x = control * cos(theta(i) + pi/2);
        arrow_y = control * sin(theta(i) + pi/2);
        
        % Update arrow
        set(arrow, 'XData', x_pos, 'YData', y_pos, 'UData', arrow_x, 'VData', arrow_y);
        
        % Update LQR pendulum if included
        if include_lqr
            % Check if we have interpolated LQR data or original data
            if exist('theta_lqr_interp', 'var')
                current_theta_lqr = theta_lqr_interp(i);
            else
                current_theta_lqr = theta_lqr(i);
            end
            
            % Calculate LQR pendulum position
            x_pos_lqr = l * sin(current_theta_lqr);
            y_pos_lqr = l * cos(current_theta_lqr);
            
            % Update LQR rod and bob
            set(rod_lqr, 'XData', [0, x_pos_lqr], 'YData', [0, y_pos_lqr]);
            set(bob_lqr, 'XData', x_pos_lqr, 'YData', y_pos_lqr);
        end
        
        % Update text
        set(time_text, 'String', ['Time: ', num2str(t(i), '%.2f'), ' s']);
        set(control_text, 'String', ['Control: ', num2str(control_signal(i), '%.2f')]);
        
        % Pause to control animation speed
        pause(0.01);
        
        % Refresh display
        drawnow;
    end
end

%% Run Simulation with PID Controller
% Create time points for solver and plotting
t_span = t_start:dt:t_end;

% Solve the ODE system
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6);
[t, x] = ode45(@(t, x) pendulum_dynamics_pid(t, x, K_p, K_i, K_d, setpoint, m, l, g, b, disturbance_time, disturbance_magnitude, dt), t_span, x0, options);

% Extract the results
theta = x(:, 1);
omega = x(:, 2);
error_integral = x(:, 3);

% Calculate the control signal at each time point
control_signal = zeros(size(t));
for i = 1:length(t)
    control_signal(i) = pid_controller(t(i), x(i, :)', K_p, K_i, K_d, setpoint, disturbance_time, disturbance_magnitude, dt);
end

% Calculate error
error = setpoint - theta;

%% Run LQR Controller (Ground Truth) if enabled
if include_lqr
    % Define the linearized pendulum system around the upright equilibrium
    % x1 = theta, x2 = omega
    % Linearized around upright position (theta = 0)
    A = [0, 1; m*g*l/(m*l^2), -b/(m*l^2)];  % System matrix
    B = [0; 1/(m*l^2)];                    % Input matrix
    
    % LQR weight matrices
    Q = diag([10, 1]);      % State penalty (prioritize angle over angular velocity)
    R = 0.1;                % Control effort penalty
    
    % Compute LQR gain
    [K_lqr, ~, ~] = lqr(A, B, Q, R);
    
    % Initial conditions for LQR (angle and angular velocity only)
    x0_lqr = [x0(1); x0(2)];  % Same initial angle as PID
    
    % Simulate LQR system
    [t_lqr, x_lqr] = ode45(@(t, x) pendulum_dynamics_lqr(t, x, K_lqr, m, l, g, b, setpoint), t_span, x0_lqr, options);
    
    % Extract results
    theta_lqr = x_lqr(:, 1);
    omega_lqr = x_lqr(:, 2);
    
    % Calculate control signal
    control_signal_lqr = zeros(size(t_lqr));
    for i = 1:length(t_lqr)
        control_signal_lqr(i) = -K_lqr * (x_lqr(i, :)' - [setpoint; 0]);
    end
    
    % Calculate error
    error_lqr = setpoint - theta_lqr;
    
    % Calculate LQR performance metrics
    % For pendulum, we'll consider "success" as reaching and staying near the upright position
    % First check if pendulum reaches upright position within tolerance
    upright_threshold = 0.1;  % Within 0.1 rad of upright
    upright_indices_lqr = find(abs(theta_lqr - setpoint) <= upright_threshold);
    
    if ~isempty(upright_indices_lqr)
        % Get time to first reach upright position
        time_to_upright_lqr = t_lqr(upright_indices_lqr(1));
        
        % Check if it stays upright
        stays_upright_lqr = false;
        for i = upright_indices_lqr(1):length(t_lqr)
            if all(abs(theta_lqr(i:end) - setpoint) <= upright_threshold)
                stays_upright_lqr = true;
                stabilization_time_lqr = t_lqr(i);
                break;
            end
        end
    else
        time_to_upright_lqr = Inf;
        stays_upright_lqr = false;
        stabilization_time_lqr = Inf;
    end
    
    % Maximum deviation from upright after stabilization
    if stays_upright_lqr
        post_stabilization_indices = find(t_lqr >= stabilization_time_lqr);
        max_deviation_lqr = max(abs(theta_lqr(post_stabilization_indices) - setpoint));
    else
        max_deviation_lqr = Inf;
    end
    
    % Steady-state error
    steady_state_error_lqr = abs(setpoint - theta_lqr(end));
end

%% Calculate Performance Metrics for PID
% For pendulum, we'll consider "success" as reaching and staying near the upright position
% First check if pendulum reaches upright position within tolerance
upright_threshold = 0.1;  % Within 0.1 rad of upright
upright_indices = find(abs(theta - setpoint) <= upright_threshold);

if ~isempty(upright_indices)
    % Get time to first reach upright position
    time_to_upright = t(upright_indices(1));
    
    % Check if it stays upright
    stays_upright = false;
    for i = upright_indices(1):length(t)
        if all(abs(theta(i:end) - setpoint) <= upright_threshold)
            stays_upright = true;
            stabilization_time = t(i);
            break;
        end
    end
else
    time_to_upright = Inf;
    stays_upright = false;
    stabilization_time = Inf;
end

% Maximum deviation from upright after stabilization
if stays_upright
    post_stabilization_indices = find(t >= stabilization_time);
    max_deviation = max(abs(theta(post_stabilization_indices) - setpoint));
else
    max_deviation = Inf;
end

% Steady-state error
steady_state_error = abs(setpoint - theta(end));

%% Plot Results
figure('Name', 'Pendulum PID Control', 'NumberTitle', 'off', 'Position', [100, 100, 800, 600]);

% Plot angle vs time
subplot(3, 1, 1);
plot(t, theta, 'b-', 'LineWidth', 2);
hold on;
plot(t, setpoint*ones(size(t)), 'r--', 'LineWidth', 1);
if include_lqr
    plot(t_lqr, theta_lqr, 'g-', 'LineWidth', 1.5);
end
if disturbance_magnitude ~= 0
    xline(disturbance_time, 'k--', 'Disturbance');
end
% Add indicators for upright position (0) and downward position (±π)
yline(0, 'r:', 'Upright');
yline(pi, 'k:', 'Downward');
yline(-pi, 'k:', 'Downward');
xlabel('Time (s)');
ylabel('Angle (rad)');
grid on;
if include_lqr
    legend('PID Control', 'Setpoint', 'Ground Truth (LQR)');
else
    legend('Angle', 'Setpoint');
end
title('System Response');

% Plot control signal vs time
subplot(3, 1, 2);
plot(t, control_signal, 'b-', 'LineWidth', 2);
hold on;
if include_lqr
    plot(t_lqr, control_signal_lqr, 'g-', 'LineWidth', 1.5);
    legend('PID Control', 'Ground Truth (LQR)');
end
xlabel('Time (s)');
ylabel('Control Signal (Torque)');
grid on;
title('Control Signal');

% Plot phase portrait (theta vs omega)
subplot(3, 1, 3);
plot(theta, omega, 'b-', 'LineWidth', 2);
hold on;
if include_lqr
    plot(theta_lqr, omega_lqr, 'g-', 'LineWidth', 1.5);
    legend('PID Control', 'Ground Truth (LQR)');
end
xlabel('Angle (rad)');
ylabel('Angular Velocity (rad/s)');
grid on;
title('Phase Portrait');

sgtitle(['Pendulum Control with PID (K_p=', num2str(K_p), ', K_i=', num2str(K_i), ', K_d=', num2str(K_d), ')'], 'FontSize', 14);

% Print performance metrics
fprintf('\nPID Performance Metrics:\n');
if ~isinf(time_to_upright)
    fprintf('Time to reach upright position: %.2f s\n', time_to_upright);
    if stays_upright
        fprintf('Stabilization time: %.2f s\n', stabilization_time);
        fprintf('Maximum deviation after stabilization: %.4f rad\n', max_deviation);
    else
        fprintf('Pendulum reached upright but did not stabilize\n');
    end
else
    fprintf('Pendulum failed to reach upright position\n');
end
fprintf('Final angle: %.4f rad\n', theta(end));
fprintf('Steady State Error: %.6f rad\n', steady_state_error);

if include_lqr
    fprintf('\nGround Truth (LQR) Performance Metrics:\n');
    if ~isinf(time_to_upright_lqr)
        fprintf('Time to reach upright position: %.2f s\n', time_to_upright_lqr);
        if stays_upright_lqr
            fprintf('Stabilization time: %.2f s\n', stabilization_time_lqr);
            fprintf('Maximum deviation after stabilization: %.4f rad\n', max_deviation_lqr);
        else
            fprintf('Pendulum reached upright but did not stabilize\n');
        end
    else
        fprintf('Pendulum failed to reach upright position\n');
    end
    fprintf('Final angle: %.4f rad\n', theta_lqr(end));
    fprintf('Steady State Error: %.6f rad\n', steady_state_error_lqr);
end

%% Create animation if requested
if create_animation
    animate_pendulum(t, theta, control_signal, l, include_lqr, t_lqr, theta_lqr, control_signal_lqr);
end