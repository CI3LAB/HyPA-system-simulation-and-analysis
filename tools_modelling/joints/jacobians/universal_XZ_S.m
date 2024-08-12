% function that computes the velocity mapping matrix S of a universal XZ joint
%   input: q = [alpha; gamma] where alpha and beta are the Euler angles along X and Z axes, respectively
%          v = [alpha_dot; gamma_dot] where alpha_dot and gamma_dot are time derivatives
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S, S_deriv] = universal_XZ_S(q, v)

    S = zeros(6,2);
    cg = cos(q(2));
    sg = sin(q(2));
    S(1:3,1:2) = [cg 0; -sg 0; 0 1];

    S_deriv = zeros(6,2);
    g_dot = v(2);
    cg_dot = -sg*g_dot;
    sg_dot = cg*g_dot;
    S_deriv(1:3,1:2) = [cg_dot 0; -sg_dot 0; 0 0];

end