% function that computes the velocity mapping matrix S of a linear Y joint
%   input: q = [y]
%          v = [y_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v


function [S_quad] = linear_Y_S_quad(~)

    S_quad = zeros(6,1,1);

end