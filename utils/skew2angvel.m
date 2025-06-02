function [ret] = skew2angvel(w)
% angvel2skew maps the 3x3 skew-symmetric matrix w_hat to the 3-vector w
%   w: 3x3 skew-symmetric matrix, ret: 3x1 column vector

w_dim = size(w);

if w_dim(1) == 3
    if numel(w_dim) == 2
        w = reshape(w, 9, 1);
    else
        w = reshape(w, 9, []);
    end
    ret = [w(6, :); w(7, :); w(2, :)];
else
    if numel(w_dim) == 2
        w = reshape(w, 4, 1);
    else
        w = reshape(w, 4, []);
    end
    ret = w(2, :);
end

end

