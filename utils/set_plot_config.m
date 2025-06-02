function config = set_plot_config(config, ratio, column)

set(0, 'defaultfigurecolor',[1 1 1]);
set(0, 'DefaultAxesXGrid','off','DefaultAxesYGrid','off')
set(0, 'defaulttextinterpreter','latex');
set(0, 'defaultAxesTickLabelInterpreter','latex');
set(0, 'defaultLegendInterpreter','latex');

config.ccf_line_width = set_line_width(ratio, column, 0.5);
config.font_size = set_font_size(ratio, column);
config.line_width = set_line_width(ratio, column);
config.marker_size = set_marker_size(ratio, column);

end