% function that computes the derivative of q
%   input: q = [y; z]
%          v = [y_dot; z_dot]

function q_derivative = linear_YZ_qderiv(~,v)
    % nothing special
    q_derivative = v;
end