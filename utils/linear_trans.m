function [ret] = linear_trans(vec)
% linear_trans maps a 2D or 3D vector to a homogeneous transformation matrix
%   vec: 2x1 or 3x1 column vector, ret: 3x3 or 4x4 homogeneous transformation matrix

ret = [eye(numel(vec)), vec;
    zeros(1, numel(vec)), 1];

end