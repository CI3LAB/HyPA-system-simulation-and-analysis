% function that computes the velocity mapping matrix S of a linear X joint
%   input: q = [x]
%          v = [x_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S_quad] = linear_X_S_quad(~)

    S_quad = zeros(6,1,1);

end