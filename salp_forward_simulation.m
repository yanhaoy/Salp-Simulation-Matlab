function salp_forward_simulation(sys)
%SALP_FORWARD_SIMULATION   Simulate and plot the forward dynamics of the salp system.
%
%   salp_forward_simulation(sys) runs a forward simulation and plots the results.

[n, m] = deal(sys.config.n, sys.config.m);

T = 8 / sys.control_handle.omega;
x0 = zeros(n + m, 1);

sol = ode45(@(t, x) x_dot_func(t, x, sys), ...
    [0, T], x0, ...
    odeset('RelTol', 1e-6, 'AbsTol', 1e-6));
t = linspace(0, T, 60 * T + 1);
x = reshape(deval(sol, t), n + m, []);

% Switch back to average link frame instead of average wheel frame
x(1:3, :) = rotation_trans(sum(sys.config.wheel_transform.rotation) / 3) * x(1:3, :);
x(3, :) = x(3, :) + sum(sys.config.wheel_transform.rotation) / 3;

%% Plot Position and Orientation

sys.config = set_plot_config(sys.config, 'fullscreen', 1);
f = create_fig('fullscreen');
ax(1) = subplot(1, 2, 1);
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

ax(2) = subplot(1, 2, 2);
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

end

function x_dot = x_dot_func(t, x, sys)
%X_DOT_FUNC   Compute the state derivative for the salp forward simulation.

[n, m] = deal(sys.config.n, sys.config.m);

g = x(1:n);
r = x(n+1:n+m);

u = sys.control_handle.u_bar + ...
    sys.control_handle.A_sin * sin(2 * pi * sys.control_handle.omega * t) + ...
    sys.control_handle.A_cos * cos(2 * pi * sys.control_handle.omega * t);

q_dot = full(sys.symbolic_handle.q_dot_velocity_func(r, u));
[g_circ, r_dot] = deal(q_dot(1:n), q_dot(n+1:n+m));

g_dot = rotation_trans(g(3)) * g_circ;

x_dot = [g_dot; r_dot];

end