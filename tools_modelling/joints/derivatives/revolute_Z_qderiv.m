% function that computes the derivative of q
%   input: q = [gamma] where gamma is the Euler angle along Z axis
%          v = [gamma_dot]

function q_derivative = revolute_Z_qderiv(~,v)
    % nothing special
    q_derivative = v;
end