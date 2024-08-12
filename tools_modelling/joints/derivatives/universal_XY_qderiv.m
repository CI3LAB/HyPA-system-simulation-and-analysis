% function that computes the derivative of q
%   input: q = [alpha; beta] where alpha and beta are the Euler angles along X, Y and Z axes, respectively
%          v = [alpha_dot; beta_dot] where alpha_dot and beta_dot are time derivatives

function q_derivative = universal_XY_qderiv(~,v)
    % nothing special
    q_derivative = v;
end