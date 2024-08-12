% function that computes the derivative of q
%   input: q = [x; z]
%          v = [x_dot; z_dot]

function q_derivative = linear_XZ_qderiv(~,v)
    % nothing special
    q_derivative = v;
end