function font_size = set_font_size(ratio, column)

default_font_size = 12;

if strcmp(ratio, 'square')
    scale = 1;
elseif strcmp(ratio, 'fullscreen')
    scale = 16/9;
end

font_size = default_font_size*scale*column;
set(0, 'DefaultAxesFontSize', font_size);

end