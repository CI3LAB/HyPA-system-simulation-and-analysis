% function that computes the velocity mapping matrix S of a linear XY joint
%   input: q = [x; y]
%          v = [x_dot; y_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S_quad] = linear_XY_S_quad(~)

    S_quad = zeros(6,2,2);

end