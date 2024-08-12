% function that computes the velocity mapping matrix S of a linear Z joint
%   input: q = [z]
%          v = [z_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S, S_deriv] = linear_Z_S(q, ~)

    R = linear_Z_R(q);

    S = zeros(6,1);
    S(4:6,1) = R*[0; 0; 1];

    S_deriv = zeros(6,1);

end