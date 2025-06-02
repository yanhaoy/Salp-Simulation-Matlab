function line_width = set_line_width(ratio, column, varargin)

if nargin == 2
    default_line_width = 1.5;
else
    default_line_width = varargin{1};
end

if strcmp(ratio, 'square')
    scale = 1;
elseif strcmp(ratio, 'fullscreen')
    scale = 16/9;
end

line_width = default_line_width*scale*column;
set(0, 'DefaultLineLineWidth', line_width);

end