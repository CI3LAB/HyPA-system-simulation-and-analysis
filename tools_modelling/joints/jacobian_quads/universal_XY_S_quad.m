% function that computes the velocity mapping matrix S of a universal XY joint
%   input: q = [alpha; beta] where alpha and beta are the Euler angles along X, Y and Z axes, respectively
%          v = [alpha_dot; beta_dot] where alpha_dot and beta_dot are time derivatives
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S_quad] = universal_XY_S_quad(q)
    cb = cos(q(2));
    sb = sin(q(2));

    S_quad = zeros(6,2,2);
    cb_dot_grad = -sb;
    sb_dot_grad = cb;
    % S_quad(1:3,1:2) = [cb_dot_grad 0; 0 0; sb_dot_grad 0];
    S_quad(1:3,1:2,2) = [cb_dot_grad 0; 0 0; sb_dot_grad 0];

end