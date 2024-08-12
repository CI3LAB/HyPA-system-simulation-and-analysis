% function that computes the derivative of q
%   input: q = [x; y; gamma] where gamma is the Euler angle along Z axes
%          v = [x_dot; y_dot; gamma_dot]

function q_derivative = planar_XY_qderiv(~,v)
    % nothing special
    q_derivative = v;
end