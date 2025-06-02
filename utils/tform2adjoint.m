function [ret] = tform2adjoint(g)
% tform2adjoint maps the rigid body transformation g, in homogeneous coordinates, to the transformation adjoint matrix, Ad_g
%   g: rigid body transformation in homogeneous coordinates, 

if size(g, 1) == 4
    R=g(1:3, 1:3);
    p=g(1:3, 4);
    p_hat=angvel2skew(p);
    ret = [R, p_hat*R;
        zeros(3, 3), R];
else
    R=g(1:2, 1:2);
    p=g(1:2, 3);
    ret = [R, [p(2); -p(1)];
        0, 0, 1];
end

end

