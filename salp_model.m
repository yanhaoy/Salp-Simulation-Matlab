function sys = salp_model(sys)
%SALP_MODEL   Build the symbolic model for the salp system.
%
%   sys = SALP_MODEL(sys) returns the system struct with symbolic handles.

import casadi.*

%% Parameters

% Dimensions: n positions and m shapes
[n, m] = deal(sys.config.n, sys.config.m);

% Shape
r = SX.sym('r', [m, 1]);

% Drag coefficients
D_local = SX.sym('D_local', [n*(m+1)+m, n*(m+1)+m]);

% Physics parameters
link_length = sys.config.link_length;
wheel_transform = sys.config.wheel_transform;
imu_transform = sys.config.imu_transform;

% Control input
u = SX.sym('u', [m+1, 1]);

%% Kinematics

% Link kinematics
g_i = cell(m+1, 1);

% Start from the tail
g_i{1} = SX.eye(3);
for i = 2:m+1
    % Translate along x axis for half link length, then rotate, and translate again
    g_i{i} = g_i{i-1} * ...
        (linear_trans([link_length(i-1)/2; 0]) * ...
        rotation_trans(r(i-1)) * ...
        linear_trans([link_length(i)/2; 0]));
end

% Transform to wheel frame
g_i_wheel = cell(m+1, 1);
for i = 1:m+1
    g_i_wheel{i} = g_i{i} * ...
        linear_trans(wheel_transform.translation(:, i)) * ...
        rotation_trans(wheel_transform.rotation(i));
end

% Transform to IMU frame
g_i_imu = cell(m+1, 1);
for i = 1:m+1
    g_i_imu{i} = g_i_wheel{i} * ...
        linear_trans(imu_transform.translation(:, i)) * ...
        rotation_trans(imu_transform.rotation(i));
end

% Convert to mean position and orientation of the wheels
pos_mean = SX.zeros(2, 1);
rot_mean = 0;
for i = 1:m+1
    pos_mean = pos_mean + g_i_wheel{i}(1:2, 3) / (m+1);
    rot_mean = rot_mean + sum(r(1:i-1)) / (m+1);
end
rot_mean = rot_mean + sum(wheel_transform.rotation) / (m+1);

g_0 = linear_trans(pos_mean) * rotation_trans(rot_mean);
for i = 1:m+1
    g_i{i} = g_0 \ g_i{i};
    g_i_wheel{i} = g_0 \ g_i_wheel{i};
    g_i_imu{i} = g_0 \ g_i_imu{i};
end

% Link body velocity Jacobian
jac_g = SX.zeros((m+1)*n, n+m);
for i = 1:m+1
    tmp = jacobian(g_i{i}(:), r);
    jac_g((i-1)*n+1:i*n, 1:n) = inv(tform2adjoint(g_i{i}));
    for j = 1:m
        jac_g((i-1)*n+1:i*n, n+j) = rbvel2twist(inv(g_i{i}) * reshape(tmp(:, j), n, n));
    end
end

% Wheel body velocity Jacobian
jac_g_wheel = SX.zeros((m+1)*n, n+m);
for i = 1:m+1
    tmp = jacobian(g_i_wheel{i}(:), r);
    jac_g_wheel((i-1)*n+1:i*n, 1:n) = inv(tform2adjoint(g_i_wheel{i}));
    for j = 1:m
        jac_g_wheel((i-1)*n+1:i*n, n+j) = rbvel2twist(inv(g_i_wheel{i}) * reshape(tmp(:, j), n, n));
    end
end

%% Motion reconstruction

% Jacobian to drag
% General thrust control with viscous drag on the link and joint:
jac_drag_thrust = [jac_g; SX.zeros(m, n), eye(m)];
% LandSalp model with viscous drag on the wheel and joint:
jac_drag_velocity = [jac_g_wheel; SX.zeros(m, n), eye(m)];

% Force in coordinates due to control input
f_control_thrust = jac_g_wheel' * reshape([u, SX.zeros(m+1, 2)]', n*(m+1), 1);
f_control_velocity = jac_g_wheel' * D_local(1:3*(m+1), 1:3*(m+1)) * ...
    reshape([u, SX.zeros(m+1, 2)]', n*(m+1), 1);

% Reconstruct the motion
q_dot_thrust = -inv(-(jac_drag_thrust' * D_local * jac_drag_thrust)) * f_control_thrust;
q_dot_velocity = -inv(-(jac_drag_velocity' * D_local * jac_drag_velocity)) * f_control_velocity;

% Create function handles
tmp = Function('q_dot_thrust', {r, u, D_local}, {q_dot_thrust}, struct('cse', true));
sys.symbolic_handle.q_dot_thrust_func = Function('q_dot_thrust', {r, u}, {tmp(r, u, sys.config.D_local)}, struct('cse', true));
tmp = Function('q_dot_velocity', {r, u, D_local}, {q_dot_velocity}, struct('cse', true));
sys.symbolic_handle.q_dot_velocity_func = Function('q_dot_velocity', {r, u}, {tmp(r, u, sys.config.D_local)}, struct('cse', true));
sys.symbolic_handle.g_i_func = Function('g_i_func', {r}, g_i, struct('cse', true));

end