% function that computes the derivative of q
%   input: q = [alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%          v = [alpha_dot; beta_dot; gamma_dot] where alpha_dot, beta_dot and gamma_dot are time derivatives

function q_derivative = spherical_eulerZYX_qderiv(~,v)
    % nothing special
    q_derivative = v;
end