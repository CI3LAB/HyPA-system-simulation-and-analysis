% function that computes the derivative of q
%   input: q = [alpha; gamma] where alpha and beta are the Euler angles along X, Z axes, respectively
%          v = [alpha_dot; gamma_dot] where alpha_dot and gamma_dot are time derivatives

function q_derivative = universal_XZ_qderiv(~,v)
    % nothing special
    q_derivative = v;
end