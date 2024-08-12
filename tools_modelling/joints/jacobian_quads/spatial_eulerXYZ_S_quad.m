% function that computes the velocity mapping matrix S of a spatial joint
%   input: q = [pos; alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%                                        and pos is the position coordinate in the PARENT FRAME
%          v = [pos_dot; alpha_dot; beta_dot; gamma_dot] where alpha_dot, beta_dot and gamma_dot are time derivatives
%                                                        and pos_dot is the linear velocity defined in the PARENT FRAME
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S_quad] = spatial_eulerXYZ_S_quad(q)

    R = spatial_eulerXYZ_R(q);

    cb = cos(q(5));
    sb = sin(q(5));
    cg = cos(q(6));
    sg = sin(q(6));

    S_quad = zeros(6,6,6);
    cb_dot_grad = -sb;
    sb_dot_grad = cb;
    cg_dot_grad = -sg;
    sg_dot_grad = cg;
    % S_deriv(1:3,4:6) = [cb_dot_grad*cg+cb*cg_dot_grad sg_dot_grad 0; -cb_dot_grad*sg-cb*sg_dot_grad cg_dot_grad 0; sb_dot_grad 0 0];
    S_quad(1:3,4:6,5) = [cb_dot_grad*cg 0 0; -cb_dot_grad*sg 0 0; sb_dot_grad 0 0];
    S_quad(1:3,4:6,6) = [cb*cg_dot_grad sg_dot_grad 0; -cb*sg_dot_grad cg_dot_grad 0; 0 0 0];
    % S_deriv(4:6,1:3) = vecX3D(-S(1:3,4:6)*v(4:6))*R;
    S_quad(4:6,1:3,4) = vecX3D([-cb*cg; cb*sg; -sb])*R; % this is effectively R_dot
    S_quad(4:6,1:3,5) = vecX3D([-sg; -cg; 0])*R;
    S_quad(4:6,1:3,6) = vecX3D([0; 0; -1])*R;

end