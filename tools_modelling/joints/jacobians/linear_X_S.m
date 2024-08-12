% function that computes the velocity mapping matrix S of a linear X joint
%   input: q = [x]
%          v = [x_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S, S_deriv] = linear_X_S(q, ~)

    R = linear_X_R(q);

    S = zeros(6,1);
    S(4:6,1) = R*[1; 0; 0];

    S_deriv = zeros(6,1);

end