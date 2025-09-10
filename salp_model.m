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

% Inertia coefficients
M_local = SX.sym('M_local', [n*(m+1), n*(m+1)]);

% Physics parameters
link_length = sys.config.link_length;
wheel_transform = sys.config.wheel_transform;
imu_transform = sys.config.imu_transform;

% Control input
u = SX.sym('u', [m+1, 1]);
u_dot = SX.sym('u_dot', [m+1, 1]);

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

% IMU body velocity Jacobian
jac_g_imu = SX.zeros((m+1)*n, n+m);
for i = 1:m+1
    tmp = jacobian(g_i_imu{i}(:), r);
    jac_g_imu((i-1)*n+1:i*n, 1:n) = inv(tform2adjoint(g_i_imu{i}));
    for j = 1:m
        jac_g_imu((i-1)*n+1:i*n, n+j) = rbvel2twist(inv(g_i_imu{i}) * reshape(tmp(:, j), n, n));
    end
end

%% Motion reconstruction

% Jacobian to drag
% General thrust control with viscous drag on the link and joint:
% jac_drag = [jac_g; SX.zeros(m, n), eye(m)];
% LandSalp model with viscous drag on the wheel and joint:
jac_drag = [jac_g_wheel; SX.zeros(m, n), eye(m)];

% Force in coordinates due to control input
f_control_thrust = jac_g_wheel' * reshape([u, SX.zeros(m+1, 2)]', n*(m+1), 1);
f_control_velocity = jac_g_wheel' * D_local(1:3*(m+1), 1:3*(m+1)) * ...
    reshape([u, SX.zeros(m+1, 2)]', n*(m+1), 1);

% Reconstruct the motion
q_dot_thrust = -inv(-(jac_drag' * D_local * jac_drag)) * f_control_thrust;
q_dot_velocity = -inv(-(jac_drag' * D_local * jac_drag)) * f_control_velocity;

q_ddot_fst_sol_thrust = jacobian(q_dot_thrust, [r; u]) * [q_dot_thrust(n+1:end); u_dot];
q_ddot_fst_sol_velocity = jacobian(q_dot_velocity, [r; u]) * [q_dot_velocity(n+1:end); u_dot];

% Reconstruct the motion considering the inertia
q_dot = SX.sym('q_dot', [n+m, 1]);
q_ddot = SX.sym('q_ddot', [n+m, 1]);

M = jac_g' * M_local * jac_g;
p = M * q_dot;
f_drag = -jac_drag' * D_local * jac_drag * q_dot;
L = q_dot' * M * q_dot / 2;

D = jac_drag' * D_local * jac_drag;
tau = if_else(norm(q_dot) < 1e-6, trace(M)/trace(D), (q_dot' * M * q_dot) / (q_dot' * D * q_dot));
f_control_thrust_lag = jac_g_wheel' * reshape([u_dot .* tau, SX.zeros(m+1, 2)]', n*(m+1), 1);
f_control_velocity_lag = jac_g_wheel' * D_local(1:3*(m+1), 1:3*(m+1)) * reshape([u_dot .* tau, SX.zeros(m+1, 2)]', n*(m+1), 1);

p_dot_sol_thrust = [(dual_lie_bracket_SE2(q_dot(1:n), p(1:n))); jacobian(L, r)'] + f_control_thrust_lag;
p_dot_sol_velocity = [(dual_lie_bracket_SE2(q_dot(1:n), p(1:n))); jacobian(L, r)'] + f_control_velocity_lag;

p_dot_sol_el_thrust = [(dual_lie_bracket_SE2(q_dot(1:n), p(1:n))); jacobian(L, r)'] + f_control_thrust + f_drag;
p_dot_sol_el_velocity = [(dual_lie_bracket_SE2(q_dot(1:n), p(1:n))); jacobian(L, r)'] + f_control_velocity + f_drag;

q_ddot_snd_sol_thrust = inv(M) * (p_dot_sol_thrust - reshape(jacobian(M, r) * q_dot(n+1:end), size(M)) * q_dot);
q_ddot_snd_sol_velocity = inv(M) * (p_dot_sol_velocity - reshape(jacobian(M, r) * q_dot(n+1:end), size(M)) * q_dot);

q_ddot_snd_sol_el_thrust = inv(M) * (p_dot_sol_el_thrust - reshape(jacobian(M, r) * q_dot(n+1:end), size(M)) * q_dot);
q_ddot_snd_sol_el_velocity = inv(M) * (p_dot_sol_el_velocity - reshape(jacobian(M, r) * q_dot(n+1:end), size(M)) * q_dot);

%% IMU body velocity and acceleration

g_circ_imu = jac_g_imu * q_dot;
g_circ_dot_imu = jacobian(g_circ_imu, [r; q_dot]) * [q_dot(n+1:end); q_ddot];
tmp = SX.zeros(n*(m+1), 1);
for i = 1:m+1
    tmp((i-1)*n+1:i*n) = rbvel2twist(twist2rbvel(g_circ_imu((i-1)*n+1:i*n)) * twist2rbvel(g_circ_imu((i-1)*n+1:i*n)));
end
g_ddot_imu_body = g_circ_dot_imu + tmp;

%% Create function handles

% From system shape, thrust force control, and drag coefficient to system body and shape velocities
sys.symbolic_handle.q_dot_thrust_unid_func = Function('q_dot_thrust', {r, u, D_local}, {q_dot_thrust}, struct('cse', true));
% From system shape and thrust force control (assuming known drag coefficient) to system body and shape velocities
sys.symbolic_handle.q_dot_thrust_func = Function('q_dot_thrust', {r, u}, {sys.symbolic_handle.q_dot_thrust_unid_func(r, u, sys.config.D_local)}, struct('cse', true));

% From system shape, thrust velocity control, and drag coefficient to system body and shape velocities
sys.symbolic_handle.q_dot_velocity_unid_func = Function('q_dot_velocity', {r, u, D_local}, {q_dot_velocity}, struct('cse', true));
% From system shape and thrust velocity control (assuming known drag coefficient) to system body and shape velocities
sys.symbolic_handle.q_dot_velocity_func = Function('q_dot_velocity', {r, u}, {sys.symbolic_handle.q_dot_velocity_unid_func(r, u, sys.config.D_local)}, struct('cse', true));

% From shape to link transformation relative to the system frame
sys.symbolic_handle.g_i_func = Function('g_i_func', {r}, g_i, struct('cse', true));

% From shape, system body and shape velocities, and inertia coefficient to system body and shape momentum
sys.symbolic_handle.p_unid_func = Function('p', {r, q_dot, M_local}, {p}, struct('cse', true));
% From shape and system body and shape velocities (assuming known inertia coefficient) to system body and shape momentum
sys.symbolic_handle.p_func = Function('p', {r, q_dot}, {sys.symbolic_handle.p_unid_func(r, q_dot, sys.config.M_local)}, struct('cse', true));

% From shape, thrust force control, the time derivative of thrust force control, and drag coefficient to the time derivative of system body and shape velocities (first-order model)
sys.symbolic_handle.q_ddot_thrust_fst_unid_func = Function('q_ddot_fst_sol_thrust', {r, u, u_dot, D_local}, {q_ddot_fst_sol_thrust}, struct('cse', true));
% From shape, thrust force control, and the time derivative of thrust force control (assuming known drag coefficient) to the time derivative of system body and shape velocities (first-order model)
sys.symbolic_handle.q_ddot_thrust_fst_func = Function('q_ddot_fst_sol_thrust', {r, u, u_dot}, {sys.symbolic_handle.q_ddot_thrust_fst_unid_func(r, u, u_dot, sys.config.D_local)}, struct('cse', true));

% From shape, thrust velocity control, the time derivative of thrust velocity control, and drag coefficient to the time derivative of system body and shape velocities (first-order model)
sys.symbolic_handle.q_ddot_velocity_fst_unid_func = Function('q_ddot_fst_sol_velocity', {r, u, u_dot, D_local}, {q_ddot_fst_sol_velocity}, struct('cse', true));
% From shape, thrust velocity control, and the time derivative of thrust velocity control (assuming known drag coefficient) to the time derivative of system body and shape velocities (first-order model)
sys.symbolic_handle.q_ddot_velocity_fst_func = Function('q_ddot_fst_sol_velocity', {r, u, u_dot}, {sys.symbolic_handle.q_ddot_velocity_fst_unid_func(r, u, u_dot, sys.config.D_local)}, struct('cse', true));

% From shape, thrust force control, the time derivative of thrust force control, drag coefficient, and inertia coefficient to the time derivative of system body and shape velocities (second-order model assuming overdamping)
sys.symbolic_handle.q_ddot_thrust_snd_unid_func = Function('q_ddot_snd_sol_thrust', {r, q_dot, u, u_dot, D_local, M_local}, {q_ddot_snd_sol_thrust}, struct('cse', true));
% From shape, thrust force control, and the time derivative of thrust force control (assuming known drag and inertia coefficients) to the time derivative of system body and shape velocities (second-order model assuming overdamping)
sys.symbolic_handle.q_ddot_thrust_snd_func = Function('q_ddot_snd_sol_thrust', {r, q_dot, u, u_dot}, {sys.symbolic_handle.q_ddot_thrust_snd_unid_func(r, q_dot, u, u_dot, sys.config.D_local, sys.config.M_local)}, struct('cse', true));

% From shape, thrust velocity control, the time derivative of thrust velocity control, drag coefficient, and inertia coefficient to the time derivative of system body and shape velocities (second-order model assuming overdamping)
sys.symbolic_handle.q_ddot_velocity_snd_unid_func = Function('q_ddot_snd_sol_velocity', {r, q_dot, u, u_dot, D_local, M_local}, {q_ddot_snd_sol_velocity}, struct('cse', true));
% From shape, thrust velocity control, and the time derivative of thrust velocity control (assuming known drag and inertia coefficients) to the time derivative of system body and shape velocities (second-order model assuming overdamping)
sys.symbolic_handle.q_ddot_velocity_snd_func = Function('q_ddot_snd_sol_velocity', {r, q_dot, u, u_dot}, {sys.symbolic_handle.q_ddot_velocity_snd_unid_func(r, q_dot, u, u_dot, sys.config.D_local, sys.config.M_local)}, struct('cse', true));

% From shape, thrust force control, the time derivative of thrust force control, drag coefficient, and inertia coefficient to the time derivative of system body and shape velocities (second-order model)
sys.symbolic_handle.q_ddot_thrust_snd_el_unid_func = Function('q_ddot_snd_sol_el_thrust', {r, q_dot, u, D_local, M_local}, {q_ddot_snd_sol_el_thrust}, struct('cse', true));
% From shape, thrust force control, and the time derivative of thrust force control (assuming known drag and inertia coefficients) to the time derivative of system body and shape velocities (second-order model)
sys.symbolic_handle.q_ddot_thrust_snd_el_func = Function('q_ddot_snd_sol_el_thrust', {r, q_dot, u}, {sys.symbolic_handle.q_ddot_thrust_snd_el_unid_func(r, q_dot, u, sys.config.D_local, sys.config.M_local)}, struct('cse', true));

% From shape, thrust velocity control, the time derivative of thrust velocity control, drag coefficient, and inertia coefficient to the time derivative of system body and shape velocities (second-order model)
sys.symbolic_handle.q_ddot_velocity_snd_el_unid_func = Function('q_ddot_snd_sol_el_velocity', {r, q_dot, u, D_local, M_local}, {q_ddot_snd_sol_el_velocity}, struct('cse', true));
% From shape, thrust velocity control, and the time derivative of thrust velocity control (assuming known drag and inertia coefficients) to the time derivative of system body and shape velocities (second-order model)
sys.symbolic_handle.q_ddot_velocity_snd_el_func = Function('q_ddot_snd_sol_el_velocity', {r, q_dot, u}, {sys.symbolic_handle.q_ddot_velocity_snd_el_unid_func(r, q_dot, u, sys.config.D_local, sys.config.M_local)}, struct('cse', true));

% From shape, system body and shape velocities, and the time derivatives of system body and shape velocities to IMU accelerations
sys.symbolic_handle.g_ddot_imu_body_func = Function('g_ddot_imu_body', {r, q_dot, q_ddot}, {g_ddot_imu_body}, struct('cse', true));

% From shape and system body and shape velocities to IMU body velocities
sys.symbolic_handle.g_circ_imu_func = Function('g_circ_imu', {r, q_dot}, {g_circ_imu}, struct('cse', true));

end