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

function [S, S_deriv] = spatial_eulerXYZ_S(q, v)

    R = spatial_eulerXYZ_R(q);

    S = zeros(6,6);
    cb = cos(q(5));
    sb = sin(q(5));
    cg = cos(q(6));
    sg = sin(q(6));
    S(1:3,4:6) = [cb*cg sg 0; -cb*sg cg 0; sb 0 1];
    S(4:6,1:3) = R;

    S_deriv = zeros(6,6);
    b_dot = v(5);
    g_dot = v(6);
    cb_dot = -sb*b_dot;
    sb_dot = cb*b_dot;
    cg_dot = -sg*g_dot;
    sg_dot = cg*g_dot;
    S_deriv(1:3,4:6) = [cb_dot*cg+cb*cg_dot sg_dot 0; -cb_dot*sg-cb*sg_dot cg_dot 0; sb_dot 0 0];
    S_deriv(4:6,1:3) = vecX3D(-S(1:3,4:6)*[v(4);v(5);v(6)])*R; % this is effectively R_dot

end