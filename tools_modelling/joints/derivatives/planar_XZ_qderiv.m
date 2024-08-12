% function that computes the derivative of q
%   input: q = [x; z; beta] where beta is the Euler angle along Y axes
%          v = [x_dot; z_dot; beta_dot]

function q_derivative = planar_XZ_qderiv(~,v)
    % nothing special
    q_derivative = v;
end