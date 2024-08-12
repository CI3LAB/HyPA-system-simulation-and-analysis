% function that computes the velocity mapping matrix S of a spherical joint
%   input: q = [alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%          v = [alpha_dot; beta_dot; gamma_dot] where alpha_dot, beta_dot and gamma_dot are time derivatives
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S_quad] = spherical_eulerXYZ_S_quad(q)

    cb = cos(q(2));
    sb = sin(q(2));
    cg = cos(q(3));
    sg = sin(q(3));

    S_quad = zeros(6,3,3);
    cb_dot_grad = -sb;
    sb_dot_grad = cb;
    cg_dot_grad = -sg;
    sg_dot_grad = cg;
    % S_quad(1:3,1:3) = [cb_dot_grad*cg+cb*cg_dot_grad sg_dot_grad 0; -cb_dot_grad*sg-cb*sg_dot_grad cg_dot_grad 0; sb_dot_grad 0 0];
    S_quad(1:3,1:3,2) = [cb_dot_grad*cg 0 0; -cb_dot_grad*sg 0 0; sb_dot_grad 0 0];
    S_quad(1:3,1:3,3) = [cb*cg_dot_grad sg_dot_grad 0; -cb*sg_dot_grad cg_dot_grad 0; 0 0 0];

end