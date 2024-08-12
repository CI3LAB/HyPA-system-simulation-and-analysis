% function that computes the derivative of q
%   input: q = [alpha] where alpha is the Euler angle along X axis
%          v = [alpha_dot]

function q_derivative = revolute_X_qderiv(~,v)
    % nothing special
    q_derivative = v;
end