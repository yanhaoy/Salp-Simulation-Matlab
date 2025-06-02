function [f, ax] = create_fig(ratio)

f = figure();
drawnow

if strcmp(ratio, 'square')
    set(f, 'Position', [49, 49, 955, 955]);
elseif strcmp(ratio, 'fullscreen')
    set(f, 'Position', [1, 49, 1920, 955]);
end

ax = axes(f);

end