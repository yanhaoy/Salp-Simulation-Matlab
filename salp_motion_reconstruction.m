function salp_motion_reconstruction(sys)
%SALP_MOTION_RECONSTRUCTION   Reconstruct and plot motion statistics from experiment data.
%
%   salp_motion_reconstruction(sys) loads processed experiment data and
%   compares predicted and experimental velocities.

%% Read Data

% Set ROS bag path
path = './data/experiment/';
% Set ROS bag index
index = 1;

% Open ROS bag
file = dir(fullfile(path, '*.bag'));
fileList = fullfile({file.folder}, {file.name});
bag_name = fileList{index};

% Control frequency (Hz)
fc = 1/6;
% Sampling frequency (Hz)
fs = 200;
% Number of cycles to analyze
if index < 3
    cycle = 15;
else
    cycle = 8;
end

load([bag_name(1:end-4), '.mat'], 'shape', 'command', 'command_dot', 'position_velocity', 'position_acceleration', 'shape_velocity', 'shape_acceleration', 'accelerometer', 'gyro', 'force', 'force_dot');

%% Compute Predicted Acceleration
[n, m] = deal(sys.config.n, sys.config.m);
q_dot_velocity = full(sys.symbolic_handle.q_dot_velocity_func(shape, command));
% q_ddot_velocity = full(sys.symbolic_handle.q_ddot_velocity_fst_func(shape, command, command_dot));
q_ddot_velocity = full(sys.symbolic_handle.q_ddot_velocity_snd_func(shape, [position_velocity; shape_velocity], command, command_dot));

g_circ_imu_mocap = reshape(full(sys.symbolic_handle.g_circ_imu_func(shape, [position_velocity; shape_velocity])),  3, 3, []);
g_circ_imu_model = reshape(full(sys.symbolic_handle.g_circ_imu_func(shape, q_dot_velocity)),  3, 3, []);
g_ddot_imu_body_mocap = reshape(full(sys.symbolic_handle.g_ddot_imu_body_func(shape, [position_velocity; shape_velocity], [position_acceleration; shape_acceleration])), 3, 3, []);
g_ddot_imu_body_model = reshape(full(sys.symbolic_handle.g_ddot_imu_body_func(shape, [position_velocity; shape_velocity], q_ddot_velocity)), 3, 3, []);

f_link_mocap = full(sys.symbolic_handle.f_link_func(shape, q_dot_velocity, [position_acceleration; shape_acceleration]));
f_link_model = full(sys.symbolic_handle.f_link_func(shape, q_dot_velocity, q_ddot_velocity));

accelerometer = accelerometer([3, 1, 2, 6, 4, 5, 9, 7, 8], :);
gyro = gyro([3, 1, 2, 6, 4, 5, 9, 7, 8], :);
imu = reshape([accelerometer([1, 2], :); gyro(3, :); accelerometer([4, 5], :); gyro(6, :); accelerometer([7, 8], :); gyro(9, :)], 3, 3, []);

%% Select Cycles for Statistics
if rem(size(q_ddot_velocity, 2), 2) == 0
    idx = [round(size(q_ddot_velocity, 2)/2) - cycle/2*fs*(1/fc) + 1 : round(size(q_ddot_velocity, 2)/2), ...
           round(size(q_ddot_velocity, 2)/2) + 1 : round(size(q_ddot_velocity, 2)/2) + cycle/2*fs*(1/fc)];
else
    idx = [round(size(q_ddot_velocity, 2)/2) - cycle/2*fs*(1/fc) : round(size(q_ddot_velocity, 2)/2), ...
           round(size(q_ddot_velocity, 2)/2) + 1 : round(size(q_ddot_velocity, 2)/2) + cycle/2*fs*(1/fc) - 1];
end

q_ddot_velocity_stat = [mean(reshape(q_ddot_velocity(:, idx), n+m, fs*(1/fc), []), 3);
    mean(reshape(q_ddot_velocity(:, idx), n+m, fs*(1/fc), []), 3) + std(reshape(q_ddot_velocity(:, idx), n+m, fs*(1/fc), []), [], 3);
    mean(reshape(q_ddot_velocity(:, idx), n+m, fs*(1/fc), []), 3) - std(reshape(q_ddot_velocity(:, idx), n+m, fs*(1/fc), []), [], 3)];
position_acceleration_stat = [mean(reshape(position_acceleration(:, idx), n, fs*(1/fc), []), 3);
    mean(reshape(position_acceleration(:, idx), n, fs*(1/fc), []), 3) + std(reshape(position_acceleration(:, idx), n, fs*(1/fc), []), [], 3);
    mean(reshape(position_acceleration(:, idx), n, fs*(1/fc), []), 3) - std(reshape(position_acceleration(:, idx), n, fs*(1/fc), []), [], 3)];
shape_acceleration_stat = [mean(reshape(shape_acceleration(:, idx), m, fs*(1/fc), []), 3);
    mean(reshape(shape_acceleration(:, idx), m, fs*(1/fc), []), 3) + std(reshape(shape_acceleration(:, idx), m, fs*(1/fc), []), [], 3);
    mean(reshape(shape_acceleration(:, idx), m, fs*(1/fc), []), 3) - std(reshape(shape_acceleration(:, idx), m, fs*(1/fc), []), [], 3)];
g_circ_imu_mocap_stat = zeros(9, 3, size(shape_acceleration_stat, 2));
g_circ_imu_model_stat = zeros(9, 3, size(shape_acceleration_stat, 2));
g_ddot_imu_body_model_stat = zeros(9, 3, size(shape_acceleration_stat, 2));
g_ddot_imu_body_mocap_stat = zeros(9, 3, size(shape_acceleration_stat, 2));
imu_stat = zeros(9, 3, size(shape_acceleration_stat, 2));
for i = 1:3
    g_circ_imu_mocap_stat(:, i, :) = [mean(reshape(g_circ_imu_mocap(:, i, idx), 3, fs*(1/fc), []), 3);
        mean(reshape(g_circ_imu_mocap(:, i, idx), 3, fs*(1/fc), []), 3) + std(reshape(g_circ_imu_mocap(:, i, idx), 3, fs*(1/fc), []), [], 3);
        mean(reshape(g_circ_imu_mocap(:, i, idx), 3, fs*(1/fc), []), 3) - std(reshape(g_circ_imu_mocap(:, i, idx), 3, fs*(1/fc), []), [], 3)];
    g_circ_imu_model_stat(:, i, :) = [mean(reshape(g_circ_imu_model(:, i, idx), 3, fs*(1/fc), []), 3);
        mean(reshape(g_circ_imu_model(:, i, idx), 3, fs*(1/fc), []), 3) + std(reshape(g_circ_imu_model(:, i, idx), 3, fs*(1/fc), []), [], 3);
        mean(reshape(g_circ_imu_model(:, i, idx), 3, fs*(1/fc), []), 3) - std(reshape(g_circ_imu_model(:, i, idx), 3, fs*(1/fc), []), [], 3)];
    g_ddot_imu_body_model_stat(:, i, :) = [mean(reshape(g_ddot_imu_body_model(:, i, idx), 3, fs*(1/fc), []), 3);
        mean(reshape(g_ddot_imu_body_model(:, i, idx), 3, fs*(1/fc), []), 3) + std(reshape(g_ddot_imu_body_model(:, i, idx), 3, fs*(1/fc), []), [], 3);
        mean(reshape(g_ddot_imu_body_model(:, i, idx), 3, fs*(1/fc), []), 3) - std(reshape(g_ddot_imu_body_model(:, i, idx), 3, fs*(1/fc), []), [], 3)];
    g_ddot_imu_body_mocap_stat(:, i, :) = [mean(reshape(g_ddot_imu_body_mocap(:, i, idx), 3, fs*(1/fc), []), 3);
        mean(reshape(g_ddot_imu_body_mocap(:, i, idx), 3, fs*(1/fc), []), 3) + std(reshape(g_ddot_imu_body_mocap(:, i, idx), 3, fs*(1/fc), []), [], 3);
        mean(reshape(g_ddot_imu_body_mocap(:, i, idx), 3, fs*(1/fc), []), 3) - std(reshape(g_ddot_imu_body_mocap(:, i, idx), 3, fs*(1/fc), []), [], 3)];
    imu_stat(:, i, :) = [mean(reshape(imu(:, i, idx), 3, fs*(1/fc), []), 3);
        mean(reshape(imu(:, i, idx), 3, fs*(1/fc), []), 3) + std(reshape(imu(:, i, idx), 3, fs*(1/fc), []), [], 3);
        mean(reshape(imu(:, i, idx), 3, fs*(1/fc), []), 3) - std(reshape(imu(:, i, idx), 3, fs*(1/fc), []), [], 3)];
end

%% Plot Translational and Angular Acceleration

sys.config = set_plot_config(sys.config, 'fullscreen', 1);
f = create_fig('fullscreen');
ax(1) = subplot(1, 2, 1);
title(ax(1), 'Position Acceleration');
hold(ax(1), 'on');
color = colororder;

yline(ax(1), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

time_stat = (0:fs*(1/fc)-1) * (1/fs);
yyaxis(ax(1), 'left');
for i = 1:2
    h_1(i) = plot(ax(1), time_stat, q_ddot_velocity_stat(i, :), 'Color', color(i, :), 'LineStyle', '--');
    h_1(i+3) = plot(ax(1), time_stat, position_acceleration_stat(i, :), 'Color', color(i, :), 'LineStyle', '-');
    patch(ax(1), [time_stat, fliplr(time_stat)], ...
        [position_acceleration_stat(i+n, :), fliplr(position_acceleration_stat(i+n*2, :))], ...
        color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);
end
yyaxis(ax(1), 'right');
i = 3;
h_1(i) = plot(ax(1), time_stat, q_ddot_velocity_stat(i, :), 'Color', color(i, :), 'LineStyle', '--');
h_1(i+3) = plot(ax(1), time_stat, position_acceleration_stat(i, :), 'Color', color(i, :), 'LineStyle', '-');
patch(ax(1), [time_stat, fliplr(time_stat)], ...
    [position_acceleration_stat(i+n, :), fliplr(position_acceleration_stat(i+n*2, :))], ...
    color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);

box(ax(1), 'on');
ax(1).YAxis(1).Color = 'k';
ax(1).YAxis(2).Color = 'k';
xlabel(ax(1), 'Time');
yyaxis(ax(1), 'left');
ylim(ax(1), 'tight');
tmp = ylim(ax(1));
ylim(ax(1), [-1, 1] * max(abs(tmp)));
ylabel(ax(1), 'Translational Acceleration $(m/s^2)$');
yyaxis(ax(1), 'right');
ylim(ax(1), 'tight');
tmp = ylim(ax(1));
ylim(ax(1), [-1, 1] * max(abs(tmp)));
ax(1).YAxis(1).Color = 'k';
ax(1).YAxis(2).Color = 'k';
ylabel(ax(1), 'Angular Acceleration $(rad/s^2)$');
axis(ax(1), 'square');
legend(h_1, {'$a^x_{\mathrm{model}}$', '$a^y_{\mathrm{model}}$', '$a^\theta_{\mathrm{model}}$', ...
    '$a^x_{\mathrm{mocap}}$', '$a^y_{\mathrm{mocap}}$', '$a^\theta_{\mathrm{mocap}}$'}, ...
    'Location', 'southoutside', 'NumColumns', 6);
hold(ax(1), 'off');

%% Plot Shape Accelerations

ax(2) = subplot(1, 2, 2);
title(ax(2), 'Shape Acceleration');
hold(ax(2), 'on');
color = circshift(colororder, 4);

yline(ax(2), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

time_stat = (0:fs*(1/fc)-1) * (1/fs);
for i = 1:2
    h_2(i) = plot(ax(2), time_stat, q_ddot_velocity_stat(i+3, :), 'Color', color(i, :), 'LineStyle', '--');
    h_2(i+2) = plot(ax(2), time_stat, shape_acceleration_stat(i, :), 'Color', color(i, :), 'LineStyle', '-');
    patch(ax(2), [time_stat, fliplr(time_stat)], ...
        [shape_acceleration_stat(i+m, :), fliplr(shape_acceleration_stat(i+m*2, :))], ...
        color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);
end

box(ax(2), 'on');
ylim(ax(2), 'tight');
tmp = ylim(ax(2));
ylim(ax(2), [-1, 1] * max(abs(tmp)));
xlabel(ax(2), 'Time');
ylabel(ax(2), 'Shape Acceleration $(rad/s^2)$');
axis(ax(2), 'square');
legend(h_2, {'$\ddot{\alpha}^{1}_{\mathrm{model}}$', '$\ddot{\alpha}^{2}_{\mathrm{model}}$', ...
    '$\ddot{\alpha}^{1}_{\mathrm{mocap}}$', '$\ddot{\alpha}^{2}_{\mathrm{mocap}}$'}, ...
    'Location', 'southoutside', 'NumColumns', 4);
hold(ax(2), 'off');
sgtitle('System Acceleration', 'FontSize', sys.config.font_size);

%% Plot IMU Readings

for j = 1:3
    sys.config = set_plot_config(sys.config, 'fullscreen', 1);
    f = create_fig('fullscreen');
    ax(2*j+1) = subplot(1, 2, 1);
    title(ax(2*j+1), 'IMU vs Mocap');
    hold(ax(2*j+1), 'on');
    color = colororder;

    yline(ax(2*j+1), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

    time_stat = (0:fs*(1/fc)-1) * (1/fs);
    yyaxis(ax(2*j+1), 'left');
    for i = 1:2
        h_1(i) = plot(ax(2*j+1), time_stat, squeeze(g_ddot_imu_body_mocap_stat(i, j, :))', 'Color', color(i, :), 'LineStyle', '--');
        h_1(i+3) = plot(ax(2*j+1), time_stat, squeeze(imu_stat(i, j, :))', 'Color', color(i, :), 'LineStyle', '-');
        patch(ax(2*j+1), [time_stat, fliplr(time_stat)], ...
            [squeeze(imu_stat(i+n, j, :))', fliplr(squeeze(imu_stat(i+n*2, j, :))')], ...
            color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);
    end
    yyaxis(ax(2*j+1), 'right');
    i = 3;
    h_1(i) = plot(ax(2*j+1), time_stat, squeeze(g_circ_imu_mocap_stat(i, j, :))', 'Color', color(i, :), 'LineStyle', '--');
    h_1(i+3) = plot(ax(2*j+1), time_stat, squeeze(imu_stat(i, j, :))', 'Color', color(i, :), 'LineStyle', '-');
    patch(ax(2*j+1), [time_stat, fliplr(time_stat)], ...
        [squeeze(imu_stat(i+n, j, :))', fliplr(squeeze(imu_stat(i+n*2, j, :))')], ...
        color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);

    box(ax(2*j+1), 'on');
    ax(2*j+1).YAxis(1).Color = 'k';
    ax(2*j+1).YAxis(2).Color = 'k';
    xlabel(ax(2*j+1), 'Time');
    yyaxis(ax(2*j+1), 'left');
    ylim(ax(2*j+1), 'tight');
    tmp = ylim(ax(2*j+1));
    ylim(ax(2*j+1), [-1, 1] * max(abs(tmp)));
    ylabel(ax(2*j+1), 'Translational Acceleration $(m/s^2)$');
    yyaxis(ax(2*j+1), 'right');
    ylim(ax(2*j+1), 'tight');
    tmp = ylim(ax(2*j+1));
    ylim(ax(2*j+1), [-1, 1] * max(abs(tmp)));
    ax(2*j+1).YAxis(1).Color = 'k';
    ax(2*j+1).YAxis(2).Color = 'k';
    ylabel(ax(2*j+1), 'Angular Velocity $(rad/s)$');
    axis(ax(2*j+1), 'square');
    legend(h_1, {'$a^x_{\mathrm{mocap}}$', '$a^y_{\mathrm{mocap}}$', '$\omega^z_{\mathrm{mocap}}$', ...
        '$a^x_{\mathrm{imu}}$', '$a^y_{\mathrm{imu}}$', '$\omega^z_{\mathrm{imu}}$'}, ...
        'Location', 'southoutside', 'NumColumns', 6);
    hold(ax(2*j+1), 'off');

    ax(2*j+2) = subplot(1, 2, 2);
    title(ax(2*j+2), 'Mocap vs Model');
    hold(ax(2*j+2), 'on');
    color = colororder;

    yline(ax(2*j+2), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

    time_stat = (0:fs*(1/fc)-1) * (1/fs);
    yyaxis(ax(2*j+2), 'left');
    for i = 1:2
        h_1(i) = plot(ax(2*j+2), time_stat, squeeze(g_ddot_imu_body_model_stat(i, j, :))', 'Color', color(i, :), 'LineStyle', '--');
        h_1(i+3) = plot(ax(2*j+2), time_stat, squeeze(g_ddot_imu_body_mocap_stat(i, j, :))', 'Color', color(i, :), 'LineStyle', '-');
        patch(ax(2*j+2), [time_stat, fliplr(time_stat)], ...
            [squeeze(g_ddot_imu_body_mocap_stat(i+n, j, :))', fliplr(squeeze(g_ddot_imu_body_mocap_stat(i+n*2, j, :))')], ...
            color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);
    end
    yyaxis(ax(2*j+2), 'right');
    i = 3;
    h_1(i) = plot(ax(2*j+2), time_stat, squeeze(g_circ_imu_model_stat(i, j, :))', 'Color', color(i, :), 'LineStyle', '--');
    h_1(i+3) = plot(ax(2*j+2), time_stat, squeeze(g_circ_imu_mocap_stat(i, j, :))', 'Color', color(i, :), 'LineStyle', '-');
    patch(ax(2*j+2), [time_stat, fliplr(time_stat)], ...
        [squeeze(g_circ_imu_mocap_stat(i+n, j, :))', fliplr(squeeze(g_circ_imu_mocap_stat(i+n*2, j, :))')], ...
        color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);

    box(ax(2*j+2), 'on');
    ax(2*j+2).YAxis(1).Color = 'k';
    ax(2*j+2).YAxis(2).Color = 'k';
    xlabel(ax(2*j+2), 'Time');
    yyaxis(ax(2*j+2), 'left');
    ylim(ax(2*j+2), 'tight');
    tmp = ylim(ax(2*j+2));
    ylim(ax(2*j+2), [-1, 1] * max(abs(tmp)));
    ylabel(ax(2*j+2), 'Translational Acceleration $(m/s^2)$');
    yyaxis(ax(2*j+2), 'right');
    ylim(ax(2*j+2), 'tight');
    tmp = ylim(ax(2*j+2));
    ylim(ax(2*j+2), [-1, 1] * max(abs(tmp)));
    ax(2*j+2).YAxis(1).Color = 'k';
    ax(2*j+2).YAxis(2).Color = 'k';
    ylabel(ax(2*j+2), 'Angular Acceleration $(rad/s^2)$');
    axis(ax(2*j+2), 'square');
    legend(h_1, {'$a^x_{\mathrm{model}}$', '$a^y_{\mathrm{model}}$', '$\omega^z_{\mathrm{model}}$', ...
        '$a^x_{\mathrm{mocap}}$', '$a^y_{\mathrm{mocap}}$', '$\omega^z_{\mathrm{mocap}}$'}, ...
        'Location', 'southoutside', 'NumColumns', 6);
    hold(ax(2*j+2), 'off');

    linkaxes(ax(2*j+1:2*j+2));
    yyaxis(ax(2*j+1), 'left');
    yyaxis(ax(2*j+2), 'left');
    linkaxes(ax(2*j+1:2*j+2));

    sgtitle(['Unit ' num2str(j)], 'FontSize', sys.config.font_size);
end

%% Compute Predicted Velocities

[n, m] = deal(sys.config.n, sys.config.m);
q_dot = full(sys.symbolic_handle.q_dot_velocity_func(shape, command));
[g_circ, r_dot] = deal(squeeze(q_dot(1:n, :, :)), squeeze(q_dot(n+1:n+m, :, :)));
g_circ = rotation_trans(sum(sys.config.wheel_transform.rotation)/3) * g_circ;
position_velocity = rotation_trans(sum(sys.config.wheel_transform.rotation)/3) * position_velocity;

%% Select Cycles for Statistics

if rem(size(g_circ, 2), 2) == 0
    idx = [round(size(g_circ, 2)/2) - cycle/2*fs*(1/fc) + 1 : round(size(g_circ, 2)/2), ...
           round(size(g_circ, 2)/2) + 1 : round(size(g_circ, 2)/2) + cycle/2*fs*(1/fc)];
else
    idx = [round(size(g_circ, 2)/2) - cycle/2*fs*(1/fc) : round(size(g_circ, 2)/2), ...
           round(size(g_circ, 2)/2) + 1 : round(size(g_circ, 2)/2) + cycle/2*fs*(1/fc) - 1];
end

g_circ_stat = [mean(reshape(g_circ(:, idx), n, fs*(1/fc), []), 3);
    mean(reshape(g_circ(:, idx), n, fs*(1/fc), []), 3) + std(reshape(g_circ(:, idx), n, fs*(1/fc), []), [], 3);
    mean(reshape(g_circ(:, idx), n, fs*(1/fc), []), 3) - std(reshape(g_circ(:, idx), n, fs*(1/fc), []), [], 3)];
r_dot_stat = [mean(reshape(r_dot(:, idx), m, fs*(1/fc), []), 3);
    mean(reshape(r_dot(:, idx), m, fs*(1/fc), []), 3) + std(reshape(r_dot(:, idx), m, fs*(1/fc), []), [], 3);
    mean(reshape(r_dot(:, idx), m, fs*(1/fc), []), 3) - std(reshape(r_dot(:, idx), m, fs*(1/fc), []), [], 3)];
position_velocity_stat = [mean(reshape(position_velocity(:, idx), n, fs*(1/fc), []), 3);
    mean(reshape(position_velocity(:, idx), n, fs*(1/fc), []), 3) + std(reshape(position_velocity(:, idx), n, fs*(1/fc), []), [], 3);
    mean(reshape(position_velocity(:, idx), n, fs*(1/fc), []), 3) - std(reshape(position_velocity(:, idx), n, fs*(1/fc), []), [], 3)];
shape_velocity_stat = [mean(reshape(shape_velocity(:, idx), m, fs*(1/fc), []), 3);
    mean(reshape(shape_velocity(:, idx), m, fs*(1/fc), []), 3) + std(reshape(shape_velocity(:, idx), m, fs*(1/fc), []), [], 3);
    mean(reshape(shape_velocity(:, idx), m, fs*(1/fc), []), 3) - std(reshape(shape_velocity(:, idx), m, fs*(1/fc), []), [], 3)];

%% Plot Translational and Angular Velocities

sys.config = set_plot_config(sys.config, 'fullscreen', 1);
f = create_fig('fullscreen');
ax(1) = subplot(1, 2, 1);
title(ax(1), 'Position Velocity');
hold(ax(1), 'on');
color = colororder;

yline(ax(1), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

time_stat = (0:fs*(1/fc)-1) * (1/fs);
yyaxis(ax(1), 'left');
for i = 1:2
    h_1(i) = plot(ax(1), time_stat, g_circ_stat(i, :), 'Color', color(i, :), 'LineStyle', '--');
    h_1(i+3) = plot(ax(1), time_stat, position_velocity_stat(i, :), 'Color', color(i, :), 'LineStyle', '-');
    patch(ax(1), [time_stat, fliplr(time_stat)], ...
        [position_velocity_stat(i+n, :), fliplr(position_velocity_stat(i+n*2, :))], ...
        color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);
end
yyaxis(ax(1), 'right');
i = 3;
h_1(i) = plot(ax(1), time_stat, g_circ_stat(i, :), 'Color', color(i, :), 'LineStyle', '--');
h_1(i+3) = plot(ax(1), time_stat, position_velocity_stat(i, :), 'Color', color(i, :), 'LineStyle', '-');
patch(ax(1), [time_stat, fliplr(time_stat)], ...
    [position_velocity_stat(i+n, :), fliplr(position_velocity_stat(i+n*2, :))], ...
    color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);

box(ax(1), 'on');
ax(1).YAxis(1).Color = 'k';
ax(1).YAxis(2).Color = 'k';
xlabel(ax(1), 'Time');
yyaxis(ax(1), 'left');
ylim(ax(1), 'tight');
tmp = ylim(ax(1));
ylim(ax(1), [-1, 1] * max(abs(tmp)));
ylabel(ax(1), 'Translational Velocity $(m/s)$');
yyaxis(ax(1), 'right');
ylim(ax(1), 'tight');
tmp = ylim(ax(1));
ylim(ax(1), [-1, 1] * max(abs(tmp)));
ax(1).YAxis(1).Color = 'k';
ax(1).YAxis(2).Color = 'k';
ylabel(ax(1), 'Angular Velocity $(rad/s)$');
axis(ax(1), 'square');
legend(h_1, {'$\xi^x_{\mathrm{model}}$', '$\xi^y_{\mathrm{model}}$', '$\xi^\theta_{\mathrm{model}}$', ...
    '$\xi^x_{\mathrm{mocap}}$', '$\xi^y_{\mathrm{mocap}}$', '$\xi^\theta_{\mathrm{mocap}}$'}, ...
    'Location', 'southoutside', 'NumColumns', 6);
hold(ax(1), 'off');

%% Plot Shape Velocities

ax(2) = subplot(1, 2, 2);
title(ax(2), 'Shape Velocity');
hold(ax(2), 'on');
color = circshift(colororder, 4);

yline(ax(2), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

time_stat = (0:fs*(1/fc)-1) * (1/fs);
for i = 1:2
    h_2(i) = plot(ax(2), time_stat, r_dot_stat(i, :), 'Color', color(i, :), 'LineStyle', '--');
    h_2(i+2) = plot(ax(2), time_stat, shape_velocity_stat(i, :), 'Color', color(i, :), 'LineStyle', '-');
    patch(ax(2), [time_stat, fliplr(time_stat)], ...
        [shape_velocity_stat(i+m, :), fliplr(shape_velocity_stat(i+m*2, :))], ...
        color(i, :), 'EdgeColor', 'none', 'FaceAlpha', 0.25);
end

box(ax(2), 'on');
ylim(ax(2), 'tight');
tmp = ylim(ax(2));
ylim(ax(2), [-1, 1] * max(abs(tmp)));
xlabel(ax(2), 'Time');
ylabel(ax(2), 'Shape Velocity $(rad/s)$');
axis(ax(2), 'square');
legend(h_2, {'$\dot{\alpha}^{1}_{\mathrm{model}}$', '$\dot{\alpha}^{2}_{\mathrm{model}}$', ...
    '$\dot{\alpha}^{1}_{\mathrm{mocap}}$', '$\dot{\alpha}^{2}_{\mathrm{mocap}}$'}, ...
    'Location', 'southoutside', 'NumColumns', 4);
hold(ax(2), 'off');
sgtitle('System Velocity', 'FontSize', sys.config.font_size);

end