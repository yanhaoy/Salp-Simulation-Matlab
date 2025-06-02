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
cycle = 15;
% cycle = 8;

load([bag_name(1:end-4), '.mat'], 'shape', 'command', 'position_velocity', 'shape_velocity');

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
legend(h_1, {'$\xi^x_{\mathrm{pred}}$', '$\xi^y_{\mathrm{pred}}$', '$\xi^\theta_{\mathrm{pred}}$', ...
    '$\xi^x_{\mathrm{exp}}$', '$\xi^y_{\mathrm{exp}}$', '$\xi^\theta_{\mathrm{exp}}$'}, ...
    'Location', 'southoutside', 'NumColumns', 6);
hold(ax(1), 'off');

%% Plot Shape Velocities

ax(2) = subplot(1, 2, 2);
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
legend(h_2, {'$\dot{\alpha}^{1}_{\mathrm{pred}}$', '$\dot{\alpha}^{2}_{\mathrm{pred}}$', ...
    '$\dot{\alpha}^{1}_{\mathrm{exp}}$', '$\dot{\alpha}^{2}_{\mathrm{exp}}$'}, ...
    'Location', 'southoutside', 'NumColumns', 4);
hold(ax(2), 'off');

end