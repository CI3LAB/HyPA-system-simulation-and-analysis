% function that computes the velocity mapping matrix S of a linear XY joint
%   input: q = [x;y]
%          v = [x_dot;y_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S, S_deriv] = linear_XY_S(q, ~)

    R = linear_XY_R(q);

    S = zeros(6,2);
    S(4:6,:) = R*[1, 0; 0, 1; 0, 0];

    S_deriv = zeros(6,2);

end