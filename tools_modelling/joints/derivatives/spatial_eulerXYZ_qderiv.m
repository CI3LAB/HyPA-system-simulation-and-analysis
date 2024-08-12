% function that computes the derivative of q
%   input: q = [pos; alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%                                        and pos is the position coordinate in the PARENT FRAME
%          v = [pos_dot; alpha_dot; beta_dot; gamma_dot] where alpha_dot, beta_dot and gamma_dot are time derivatives
%                                                        and pos_dot is the linear velocity defined in the PARENT FRAME
function q_derivative = spatial_eulerXYZ_qderiv(~, v)
    % nothing special
    q_derivative = v;
end