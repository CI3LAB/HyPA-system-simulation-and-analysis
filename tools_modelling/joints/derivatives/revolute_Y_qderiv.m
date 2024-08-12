% function that computes the derivative of q
%   input: q = [beta] where beta is the Euler angle along Y axis
%          v = [beta_dot]

function q_derivative = revolute_Y_qderiv(~,v)
    % nothing special
    q_derivative = v;
end