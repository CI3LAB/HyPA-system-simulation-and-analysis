% function that computes the derivative of q
%   input: q = [y; z; alpha] where alpha is the Euler angle along X axes
%          v = [y_dot; z_dot; alpha_dot]

function q_derivative = planar_YZ_qderiv(~,v)
    % nothing special
    q_derivative = v;
end