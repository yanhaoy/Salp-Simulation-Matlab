function [ret] = rotation_trans(theta, direction)
% rotation_trans maps a rotation angle theta to a 2D or 3D rotation matrix
%   theta: rotation angle in radians
%   direction: 'x', 'y', or 'z' (optional)


if nargin == 1
    ret = [cos(theta), -sin(theta), 0;
        sin(theta), cos(theta), 0;
        0, 0, 1];
else
    switch direction
        case 'x'
            ret = [1, 0, 0;
                0, cos(theta), -sin(theta);
                0, sin(theta), cos(theta)];
        case 'y'
            ret = [cos(theta), 0, sin(theta);
                0, 1, 0;
                -sin(theta), 0, cos(theta)];
        case 'z'
            ret = [cos(theta), -sin(theta), 0;
                sin(theta), cos(theta), 0;
                0, 0, 1];
        otherwise
    end
end

end