function marker_size = set_marker_size(ratio, column)

default_marker_size = 7.5;

if strcmp(ratio, 'square')
    scale = 1;
elseif strcmp(ratio, 'fullscreen')
    scale = 16/9;
end

marker_size = default_marker_size*scale*column;

end