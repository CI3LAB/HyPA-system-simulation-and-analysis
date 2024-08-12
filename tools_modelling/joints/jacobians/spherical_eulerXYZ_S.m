% function that computes the velocity mapping matrix S of a spherical joint
%   input: q = [alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%          v = [alpha_dot; beta_dot; gamma_dot] where alpha_dot, beta_dot and gamma_dot are time derivatives
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S, S_deriv] = spherical_eulerXYZ_S(q, v)

    S = zeros(6,3);
    cb = cos(q(2));
    sb = sin(q(2));
    cg = cos(q(3));
    sg = sin(q(3));
    S(1:3,1:3) = [cb*cg sg 0; -cb*sg cg 0; sb 0 1];

    S_deriv = zeros(6,3);
    b_dot = v(2);
    g_dot = v(3);
    cb_dot = -sb*b_dot;
    sb_dot = cb*b_dot;
    cg_dot = -sg*g_dot;
    sg_dot = cg*g_dot;
    S_deriv(1:3,1:3) = [cb_dot*cg+cb*cg_dot sg_dot 0; -cb_dot*sg-cb*sg_dot cg_dot 0; sb_dot 0 0];

end