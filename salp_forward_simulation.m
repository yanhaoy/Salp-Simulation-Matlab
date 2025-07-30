function salp_forward_simulation(sys)
%SALP_FORWARD_SIMULATION   Simulate and plot the forward dynamics of the salp system.
%
%   salp_forward_simulation(sys) runs a forward simulation and plots the results.

[n, m] = deal(sys.config.n, sys.config.m);

T = 8 / sys.control_handle.omega;
x0 = zeros(2 * (n + m), 1);

sol = ode45(@(t, x) x_dot_func(t, x, sys), ...
    [0, T], x0, ...
    odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
t = linspace(0, T, 60 * T + 1);
x = reshape(deval(sol, t), 2 * (n + m), []);

% Switch back to average link frame instead of average wheel frame
x(1:3, :) = rotation_trans(sum(sys.config.wheel_transform.rotation) / 3) * x(1:3, :);
x(3, :) = x(3, :) + sum(sys.config.wheel_transform.rotation) / 3;

%% Plot Position and Orientation

sys.config = set_plot_config(sys.config, 'square', 2);
[~, ax(1)] = create_fig('square');
hold(ax(1), 'on');
color = colororder;

yline(ax(1), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

yyaxis(ax(1), 'left');
for i = 1:2
    h_1(i) = plot(ax(1), t, x(i, :), 'Color', color(i, :), 'LineStyle', '-');
end
yyaxis(ax(1), 'right');
i = 3;
h_1(i) = plot(ax(1), t, x(i, :), 'Color', color(i, :), 'LineStyle', '-');

box(ax(1), 'on');
ax(1).YAxis(1).Color = 'k';
ax(1).YAxis(2).Color = 'k';
xlabel(ax(1), 'Time');
yyaxis(ax(1), 'left');
ylim(ax(1), 'tight');
tmp = ylim(ax(1));
ylim(ax(1), [-1, 1] * max(abs(tmp)));
ylabel(ax(1), 'Translation $(m)$');
yyaxis(ax(1), 'right');
ylim(ax(1), 'tight');
tmp = ylim(ax(1));
ylim(ax(1), [-1, 1] * max(abs(tmp)));
ax(1).YAxis(1).Color = 'k';
ax(1).YAxis(2).Color = 'k';
ylabel(ax(1), 'Rotation $(rad)$');
axis(ax(1), 'square');
legend(h_1, {'$x$', '$y$', '$\theta$'}, 'Location', 'southoutside', 'NumColumns', 3);
hold(ax(1), 'off');

%% Plot Shape Variables

[~, ax(2)] = create_fig('square');
hold(ax(2), 'on');
color = circshift(colororder, 4);

yline(ax(2), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

for i = 1:2
    h_2(i) = plot(ax(2), t, x(i + 3, :), 'Color', color(i, :), 'LineStyle', '-');
end

box(ax(2), 'on');
ylim(ax(2), 'tight');
tmp = ylim(ax(2));
ylim(ax(2), [-1, 1] * max(abs(tmp)));
xlabel(ax(2), 'Time');
ylabel(ax(2), 'Shape $(rad)$');
axis(ax(2), 'square');
legend(h_2, {'$\alpha_1$', '$\alpha_2$'}, 'Location', 'southoutside', 'NumColumns', 2);
hold(ax(2), 'off');

%% Plot Momentum

[~, ax(3)] = create_fig('square');
hold(ax(3), 'on');
color = colororder;

yline(ax(3), 0, 'Color', [0.5, 0.5, 0.5], 'LineStyle', ':', 'LineWidth', sys.config.line_width);

yyaxis(ax(3), 'left');
for i = 1:2
    h_3(i) = plot(ax(3), t, x(i + 6, :), 'Color', color(i, :), 'LineStyle', '-');
end
yyaxis(ax(3), 'right');
i = 3;
h_3(i) = plot(ax(3), t, x(i + 6, :), 'Color', color(i, :), 'LineStyle', '-');

box(ax(3), 'on');
ax(3).YAxis(1).Color = 'k';
ax(3).YAxis(2).Color = 'k';
xlabel(ax(3), 'Time');
yyaxis(ax(3), 'left');
ylim(ax(3), 'tight');
tmp = ylim(ax(3));
ylim(ax(3), [-1, 1] * max(abs(tmp)));
ylabel(ax(3), 'Linear $(kg \cdot m/s)$');
yyaxis(ax(3), 'right');
ylim(ax(3), 'tight');
tmp = ylim(ax(3));
ylim(ax(3), [-1, 1] * max(abs(tmp)));
ax(3).YAxis(1).Color = 'k';
ax(3).YAxis(2).Color = 'k';
ylabel(ax(3), 'Angular $(kg \cdot m^2/s)$');
axis(ax(3), 'square');
legend(h_3, {'$p_x$', '$p_y$', '$p_\theta$'}, 'Location', 'southoutside', 'NumColumns', 3);
hold(ax(3), 'off');

end

function x_dot = x_dot_func(t, x, sys)
%X_DOT_FUNC   Compute the state derivative for the salp forward simulation.

[n, m] = deal(sys.config.n, sys.config.m);

g = x(1:n);
r = x(n+1:n+m);
p = x(n+m+1:end);

u = sys.control_handle.u_bar + ...
    sys.control_handle.A_sin * sin(2 * pi * sys.control_handle.omega * t) + ...
    sys.control_handle.A_cos * cos(2 * pi * sys.control_handle.omega * t);

eom = full(sys.symbolic_handle.eom_velocity_func(r, p, u));
[g_circ, r_dot, p_dot] = deal(eom(1:n), eom(n+1:n+m), eom(n+m+1:end));

g_dot = rotation_trans(g(3)) * g_circ;

x_dot = [g_dot; r_dot; p_dot];

end