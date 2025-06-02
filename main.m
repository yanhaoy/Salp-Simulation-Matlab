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
sys.config.imu_transform.translation = [0.0, 0.0, 0.0; -0.0601, -0.0601, -0.0601];

% Wheel radius
sys.config.wheel_radius = 0.08255 / 2;

% Position dimension
sys.config.n = 3;

% Shape dimension
sys.config.m = numel(sys.config.link_length) - 1;

% Drag coefficients
% General thrust control with viscous drag on the link and joint:
% sys.config.D_local = diag([10, 100, 0.272^2/12*100, 10, 100, 0.272^2/12*100, 10, 100, 0.272^2/12*100, 0.01, 0.01]);
% LandSalp model with viscous drag on the wheel and joint:
sys.config.D_local = load('./data/res.mat').D_local;

%% Setup the model

sys = salp_model(sys);

%% Example: motion reconstruction

salp_motion_reconstruction(sys)

%% Example: forward simulation

sys.control_handle = load('./data/res.mat').control;
salp_forward_simulation(sys)