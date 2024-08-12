% function that computes the velocity mapping matrix S of a linear YZ joint
%   input: q = [y; z]
%          v = [y_dot; z_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S_quad] = linear_YZ_S_quad(~)

    S_quad = zeros(6,2,2);

end