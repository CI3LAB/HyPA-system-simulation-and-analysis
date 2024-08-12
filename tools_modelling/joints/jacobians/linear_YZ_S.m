% function that computes the velocity mapping matrix S of a linear YZ joint
%   input: q = [y;z]
%          v = [y_dot;z_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S, S_deriv] = linear_YZ_S(q, ~)

    R = linear_YZ_R(q);

    S = zeros(6,2);
    S(4:6,:) = R*[0, 0; 1, 0; 0, 1];

    S_deriv = zeros(6,2);

end