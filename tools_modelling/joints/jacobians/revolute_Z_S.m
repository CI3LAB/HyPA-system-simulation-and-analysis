% function that computes the velocity mapping matrix S of a revolute Z joint
%   input: q = [gamma] where gamma is the Euler angles along Z axis
%          v = [gamma_dot]
%   output: S and S_deriv with the latter being the element-wise time derivative of the former
%           v_link (in local frame) = S*v

% an additional note on rotation matrix derivative: 
% (more detailed discussion is seen in [S. Zhao, “Time Derivative of Rotation Matrices: A Tutorial,�? 2016.])
%           if x' = R x with x' the coordinate in the moving frame (rotating with angular velocity omega) and x the coordinate in the fixed frame, we have
%           R_dot = [-omega]x*R, with [-omega]x being the skew symmatric matrix of -omega.

function [S, S_deriv] = revolute_Z_S(~, ~)

    S = zeros(6,1);
    S(1:3) = [0; 0; 1];

    S_deriv = zeros(6,1);

end