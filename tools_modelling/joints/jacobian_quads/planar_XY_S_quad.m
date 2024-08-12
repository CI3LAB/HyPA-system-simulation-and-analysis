% function that computes the velocity mapping matrix S of a spatial joint
%   input: q = [x; y; \gamma] where gamma is the Euler angle along Z axis
%                                        and x and y are the position coordinates in the PARENT FRAME
%          v = [x_dot; y_dot; gamma_dot] where gamma_dot is time derivative
%                                                        and x_dot and y_dot are the linear velocity defined in the PARENT FRAME
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S_quad] = planar_XY_S_quad(q)

    R = planar_XY_R(q);

    S_quad = zeros(6,3,3);
    S_quad(4:6,1:2,3) = vecX3D([0; 0; -1])*R*[1 0; 0 1; 0 0]; % this is effectively R_dot

end