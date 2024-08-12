% function that computes the velocity mapping matrix S of a spatial joint
%   input: q = [y; z; \alpha] where alpha is the Euler angle along X axis
%                                        and y and z are the position coordinates in the PARENT FRAME
%          v = [y_dot; z_dot; alpha_dot] where alpha_dot is time derivative
%                                                        and y_dot and z_dot are the linear velocity defined in the PARENT FRAME
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S, S_deriv] = planar_YZ_S(q, v)

    R = planar_YZ_R(q);

    S = zeros(6,3);
    S(1:3,3) = [1; 0; 0];
    S(4:6,1:2) = R*[0 0; 1 0; 0 1];

    S_deriv = zeros(6,3);
    S_deriv(4:6,1:2) = vecX3D(-S(1:3,3)*v(3))*R*[0 0; 1 0; 0 1]; % this is effectively R_dot

end