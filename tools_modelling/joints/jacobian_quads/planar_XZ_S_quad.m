% function that computes the velocity mapping matrix S of a spatial joint
%   input: q = [x; z; \beta] where beta is the Euler angle along Y axis
%                                        and x and z are the position coordinates in the PARENT FRAME
%          v = [x_dot; z_dot; beta_dot] where beta_dot is time derivative
%                                                        and x_dot and z_dot are the linear velocity defined in the PARENT FRAME
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S_quad] = planar_XZ_S_quad(q)

    R = planar_XZ_R(q);

    S_quad = zeros(6,3,3);
    S_quad(4:6,1:2,3) = vecX3D([0; -1; 0])*R*[1 0; 0 0; 0 1]; % this is effectively R_dot

end