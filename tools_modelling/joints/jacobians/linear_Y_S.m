% function that computes the velocity mapping matrix S of a linear Y joint
%   input: q = [y]
%          v = [y_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S, S_deriv] = linear_Y_S(q, ~)

    R = linear_Y_R(q);

    S = zeros(6,1);
    S(4:6,1) = R*[0; 1; 0];

    S_deriv = zeros(6,1);

end