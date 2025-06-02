function salp_bag(sys)
%SALP_BAG   Process and extract data from a ROS bag for the salp system.
%
%   salp_bag(sys) reads, processes, and saves experiment data.

%% Read Data

% Set ROS bag path
% path = './data/data_collection/';
path = './data/experiment/';

% Set ROS bag index
index = 1;

% Open ROS bag
file = dir(fullfile(path, '*.bag'));
fileList = fullfile({file.folder}, {file.name});
bag_name = fileList{index};
bag = rosbag(bag_name);

% Smoothing settings
steepness = 1 - 1e-12;

% Control frequency (Hz)
fc = 1/6;

% Sampling frequency (Hz)
fs = 200;

% Read messages
msg = readMessages(select(bag, 'Topic', '/land_salp_sensing'), 'DataFormat', 'struct');
time_raw = cell2mat(cellfun(@(m) double(m.Header.Stamp.Sec) + 1e-9*double(m.Header.Stamp.Nsec), msg, 'UniformOutput', false))';
data_raw = cell2mat(cellfun(@(m) m.Data', msg, 'UniformOutput', false))';

% Resample data
ts = timeseries(data_raw', time_raw);
time_resample = time_raw(1):1/fs:time_raw(end);
ts_resample = resample(ts, time_resample);
data_resample = getdatasamples(ts_resample, 1:numel(time_resample))';

%% Extract Raw Data

% Actuation data
velocity_raw = data_resample(1:3, :);
force_raw = data_resample(4:6, :);
command_raw = data_resample(7:9, :);
accelerometer_raw = data_resample(end-17:end-9, :);
gyro_raw = data_resample(end-8:end, :);

% HEBI has reversed order of actuators
velocity_raw = velocity_raw([2, 1, 3], :);
force_raw = force_raw([2, 1, 3], :);
command_raw = command_raw([2, 1, 3], :);
accelerometer_raw = accelerometer_raw([4:6, 1:3, 7:9], :);
gyro_raw = gyro_raw([4:6, 1:3, 7:9], :);

% Compute the corresponding linear force and velocity given wheel radius
velocity_raw = velocity_raw .* sys.config.wheel_radius;
force_raw = force_raw ./ sys.config.wheel_radius;
command_raw = command_raw .* sys.config.wheel_radius;

% Mocap data
link_position_raw = data_resample(10:18, :);

% Rotate mocap data (mocap has x up and y left)
link_position_raw = blkdiag(rotation_trans(pi/2), rotation_trans(pi/2), rotation_trans(pi/2)) * link_position_raw;

% Reshape mocap data
link_position_raw = reshape(link_position_raw, 3, 3, []);

%% Solve Velocities

% Compute shape from link orientation
shape_raw = zeros(2, size(link_position_raw, 3));
for i = 1:2
    shape_raw(i, :) = wrapToPi(link_position_raw(3, i+1, :) - link_position_raw(3, i, :));
end

% Compute shape velocity
shape_velocity_raw = [shape_raw(:, 2) - shape_raw(:, 1), ...
    shape_raw(:, 3:end) - shape_raw(:, 1:end-2), ...
    shape_raw(:, end) - shape_raw(:, end-1)] ./ ...
    [time_resample(2)-time_resample(1), time_resample(3:end) - time_resample(1:end-2), time_resample(end)-time_resample(end-1)];

% Compute system position from link position
g_i = cell(1, 3);
[g_i{:}] = sys.symbolic_handle.g_i_func(shape_raw);
g_0_mat_set = zeros(3, 3, 3, size(link_position_raw, 3));
for i = 1:3
    g_i{i} = reshape(full(g_i{i}), 3, 3, []);
    g_i_inv = pageinv(g_i{i});
    g_i_world = [cos(link_position_raw(3, i, :)), -sin(link_position_raw(3, i, :)), link_position_raw(1, i, :);
        sin(link_position_raw(3, i, :)), cos(link_position_raw(3, i, :)), link_position_raw(2, i, :);
        repmat([0, 0, 1], 1, 1, size(link_position_raw, 3))];
    g_0_mat_set(:, :, i, :) = pagemtimes(g_i_world, g_i_inv);
end

% Use difference to average orientation to avoid singularity
g_0_diff_mat = pagemtimes(pageinv(g_0_mat_set(:, :, 1, :)), g_0_mat_set);
position_raw = squeeze(reshape([mean(g_0_mat_set(1:2, 3, :, :), 3);
    atan2(g_0_mat_set(2, 1, 1, :), g_0_mat_set(1, 1, 1, :)) + ...
    mean(atan2(g_0_diff_mat(2, 1, :, :), g_0_diff_mat(1, 1, :, :)), 3)], 3, 1, []));

% Compute position velocity
vel_mat_set = cat(4, ...
    pagemtimes(pageinv(g_0_mat_set(:, :, :, 1)), g_0_mat_set(:, :, :, 2)), ...
    pagemtimes(pageinv(g_0_mat_set(:, :, :, 1:end-2)), g_0_mat_set(:, :, :, 3:end)), ...
    pagemtimes(pageinv(g_0_mat_set(:, :, :, end-1)), g_0_mat_set(:, :, :, end)));
vel_set = zeros(size(vel_mat_set));
for i = 1:size(vel_mat_set, 3)
    for j = 1:size(vel_mat_set, 4)
        vel_set(:, :, i, j) = logm(vel_mat_set(:, :, i, j));
    end
end
position_velocity_raw = squeeze(mean([vel_set(1:2, 3, :, :); vel_set(2, 1, :, :)], 3)) ./ ...
    [time_resample(2)-time_resample(1), time_resample(3:end) - time_resample(1:end-2), time_resample(end)-time_resample(end-1)];

%% Process Data

% Smooth actuation force
force = lowpass(force_raw', 2*fc, fs, 'Steepness', steepness)';
velocity = lowpass(velocity_raw', 2*fc, fs, 'Steepness', steepness)';

% Smooth shape
shape = lowpass(shape_raw', 2*fc, fs, 'Steepness', steepness)';
shape_velocity = lowpass(shape_velocity_raw', 2*fc, fs, 'Steepness', steepness)';

% Smooth position
position = lowpass(position_raw', 2*fc, fs, 'Steepness', steepness)';
position_velocity = lowpass(position_velocity_raw', 2*fc, fs, 'Steepness', steepness)';

%% Extract Execution Time

idx = find(sum(abs(command_raw)) > 0);
time = time_resample(:, idx);
command = command_raw(:, idx);
position = position(:, idx);
position_velocity = position_velocity(:, idx);
shape = shape(:, idx);
shape_velocity = shape_velocity(:, idx);
force = force(:, idx);
velocity = velocity(:, idx);

%% Save Data

save([bag_name(1:end-4), '.mat'], ...
    'data_raw', ...
    'time_raw', ...
    'data_resample', ...
    'time_resample', ...
    'time', ...
    'force_raw', ...
    'force', ...
    'velocity_raw', ...
    'velocity', ...
    'shape_raw', ...
    'shape', ...
    'shape_velocity_raw', ...
    'shape_velocity', ...
    'position_raw', ...
    'position', ...
    'position_velocity_raw', ...
    'position_velocity', ...
    'command_raw', ...
    'command', ...
    'accelerometer_raw', ...
    'gyro_raw');

end