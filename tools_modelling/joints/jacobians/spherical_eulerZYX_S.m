% function that computes the velocity mapping matrix S of a spherical joint
%   input: q = [alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%          v = [alpha_dot; beta_dot; gamma_dot] where alpha_dot, beta_dot and gamma_dot are time derivatives
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S, S_deriv] = spherical_eulerZYX_S(q, v)

    S = zeros(6,3);
    ca = cos(q(1));
    sa = sin(q(2));
    cb = cos(q(2));
    sb = sin(q(2));
    S(1:3,1:3) = [1 0 -sb; 0 ca sa*cb; 0 -sa ca*cb];

    S_deriv = zeros(6,3);
    a_dot = v(1);
    b_dot = v(2);
    ca_dot = -sa*a_dot;
    sa_dot = ca*a_dot;
    cb_dot = -sb*b_dot;
    sb_dot = cb*b_dot;
    S_deriv(1:3,1:3) = [0 0 -sb_dot; 0 ca_dot sa_dot*cb+sa*cb_dot; 0 -sa_dot ca_dot*cb+ca*cb_dot];

end