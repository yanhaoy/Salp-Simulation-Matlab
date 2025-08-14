%% Initialization

% Add the path to the utils directory
addpath('./utils')

% Add the path to the CasADi and CVX libraries
addpath('D:/casadi')

%% Configuration

sys = struct();
sys.config = struct();

% Link lengths of each unit
sys.config.link_length = [0.272; 0.272; 0.272];

% Transformation from unit body frame (x aligned with major axis) to wheel frame (x aligned with jet)
sys.config.wheel_transform.rotation = deg2rad([-57, -130, -57]);
sys.config.wheel_transform.translation = [0.0140, 0.0128, 0.0140; 0.0091, -0.0107, 0.0091];

% Transformation from wheel frame to IMU frame (2D x-y plane aligned with 3D z-x plane of IMU reading)
sys.config.imu_transform.rotation = deg2rad([90, 90, 90]);
sys.config.imu_transform.translation = [0.0, 0.0, 0.0; -0.06015, -0.06015, -0.06015];

% Transformation from wheel frame to actuator frame 
actuator_transform.rotation = deg2rad([0, 0, 0]);
actuator_transform.translation = [0.0, 0.0, 0.0; -0.0446, -0.0446, -0.0446];

% Transformation from unit body frame to caster frame 
caster_transform.rotation = deg2rad([0, 0, 0]);
caster_transform.translation = [-0.028388, -0.014413, -0.028388; -0.077994, 0.081739, -0.077994];

% Wheel radius
sys.config.wheel_radius = 0.08255 / 2;

% Position dimension
sys.config.n = 3;

% Shape dimension
sys.config.m = numel(sys.config.link_length) - 1;

% Inertia coefficients
mass_actuator = 0.35;
mass_wheel = 0.177;
mass_caster = 0.316;
sys.config.M_local = zeros(sys.config.n * (sys.config.m + 1), sys.config.n * (sys.config.m + 1));
for i = 1:sys.config.m + 1
    g_wheel = linear_trans(sys.config.wheel_transform.translation(:, i)) * rotation_trans(sys.config.wheel_transform.rotation(i));
    g_actuator = g_wheel * linear_trans(actuator_transform.translation(:, i)) * rotation_trans(actuator_transform.rotation(i));
    g_caster = linear_trans(caster_transform.translation(:, i)) * rotation_trans(caster_transform.rotation(i));
    sys.config.M_local((i-1)*sys.config.n+1:i*sys.config.n, (i-1)*sys.config.n+1:i*sys.config.n) = diag([mass_actuator + mass_wheel + mass_caster; ...
        mass_actuator + mass_wheel + mass_caster; ...
        norm(g_actuator(1:2, 3))^2 * mass_actuator + ...
        norm(g_wheel(1:2, 3))^2 * mass_wheel + ...
        norm(g_caster(1:2, 3))^2 * mass_caster + ...
        1 / 4 * mass_wheel * sys.config.wheel_radius^2 + ...
        1 / 12 * mass_actuator * (0.110^2 + 0.031^2)] + ...
        mass_caster * 0.035^2);
    Sxy = mass_actuator * g_actuator(1:2, 3) + mass_wheel * g_wheel(1:2, 3) + mass_caster * g_caster(1:2, 3);
    sys.config.M_local((i-1)*sys.config.n+1:i*sys.config.n-1, i*sys.config.n) = [-Sxy(2); Sxy(1)];
    sys.config.M_local(i*sys.config.n, (i-1)*sys.config.n+1:i*sys.config.n-1) = [-Sxy(2), Sxy(1)];
end

% Drag coefficients
% General thrust control with viscous drag on the link and joint:
% sys.config.D_local = diag([10, 100, 0.272^2/12*100, 10, 100, 0.272^2/12*100, 10, 100, 0.272^2/12*100, 0.01, 0.01]);
% LandSalp model with viscous drag on the wheel and joint:
sys.config.D_local = load('./data/res.mat').D_local;
sys.config.M_local = load('./data/res.mat').M_local;

%% Setup the model

sys = salp_model(sys);

%% Example: motion reconstruction

salp_motion_reconstruction(sys)

%% Example: forward simulation

sys.control_handle = load('./data/res.mat').control;
salp_forward_simulation(sys)