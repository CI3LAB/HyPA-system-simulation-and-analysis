% function that computes the velocity mapping matrix S of a spatial joint
%   input: q = [pos; n] where n is the quaternion and pos is the position coordinate in the PARENT FRAME
%          v = [pos_dot; omega] where omega is the angular velocity defined in the LOCAL FRAME
%                               and pos_dot is the linear velocity defined in the PARENT FRAME
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S_quad] = spatial_quaternion_S_quad(q)

    R = spatial_quaternion_R(q);

    S_quad = zeros(6,6,6);
    S_quad(4:6,1:3,4) = vecX3D([-1;0;0])*R; % this is effectively R_dot
    S_quad(4:6,1:3,5) = vecX3D([0;-1;0])*R;
    S_quad(4:6,1:3,6) = vecX3D([0;0;-1])*R;

end