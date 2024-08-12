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

function [S_quad] = spatial_eulerZYX_S_quad(q)

    R = spatial_eulerZYX_R(q);

    ca = cos(q(4));
    sa = sin(q(4));
    cb = cos(q(5));
    sb = sin(q(5));

    S_quad = zeros(6,6,6);
    ca_dot_grad = -sa;
    sa_dot_grad = ca;
    cb_dot_grad = -sb;
    sb_dot_grad = cb;
    % S_deriv(1:3,4:6) = [0 0 -sb_dot_grad; 0 ca_dot_grad sa_dot_grad*cb+sa*cb_dot_grad; 0 -sa_dot_grad ca_dot_grad*cb+ca*cb_dot_grad];
    S_quad(1:3,4:6,4) = [0 0 0; 0 ca_dot_grad sa_dot_grad*cb; 0 -sa_dot_grad ca_dot_grad*cb];
    S_quad(1:3,4:6,5) = [0 0 -sb_dot_grad; 0 0 sa*cb_dot_grad; 0 0 ca*cb_dot_grad];
    % S_deriv(4:6,1:3) = vecX3D(-[1 0 -sb; 0 ca sa*cb; 0 -sa ca*cb]*v(4:6))*R;
    S_quad(4:6,1:3,4) = vecX3D([-1; 0; 0])*R; % this is effectively R_dot
    S_quad(4:6,1:3,5) = vecX3D([0; -ca; sa])*R;
    S_quad(4:6,1:3,6) = vecX3D([sb; -sa*cb; -ca*cb])*R;

end