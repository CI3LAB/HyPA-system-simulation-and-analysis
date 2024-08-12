% function that computes the velocity mapping matrix S of a spherical joint
%   input: q = [n] where n is the quaternion
%          v = [omega] where omega is the angular velocity defined in the LOCAL FRAME
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S_quad] = spherical_quaternion_S_quad(~)


    S_quad = zeros(6,3,3);

end