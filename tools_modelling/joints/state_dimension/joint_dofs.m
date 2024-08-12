% function that accepts a joint type and q, calls the corresponding function to compute the quaternion representation of the relative rotation (from parent to child)
% joint type index library:
%       1 - spatial_quaternion
%       2 - spatial_eulerXYZ
%       3 - sperical_quaternion
%       4 - sperical_eulerXYZ
%       5 - universal_XY

% eulerXYZ means that from parent frame to child frame the rotations are sequentially performed along axes X, Y and Z.

function [pos_dof, vel_dof] = joint_dofs(joint_type_index)
    switch joint_type_index
    case 1
        [pos_dof, vel_dof] = spatial_quaternion_dofs();
    case 2
        [pos_dof, vel_dof] = spatial_eulerXYZ_dofs();
    case 3
        [pos_dof, vel_dof] = spherical_quaternion_dofs();
    case 4
        [pos_dof, vel_dof] = spherical_eulerXYZ_dofs();
    case 5
        [pos_dof, vel_dof] = universal_XY_dofs();
    case 6
        [pos_dof, vel_dof] = revolute_X_dofs();
    case 7
        [pos_dof, vel_dof] = revolute_Y_dofs();
    case 8
        [pos_dof, vel_dof] = revolute_Z_dofs();
    case 9
        [pos_dof, vel_dof] = universal_XZ_dofs();
    case 10
        [pos_dof, vel_dof] = planar_XZ_dofs();
    case 11
        [pos_dof, vel_dof] = planar_XY_dofs();
    case 12
        [pos_dof, vel_dof] = planar_YZ_dofs();
    case 13
        [pos_dof, vel_dof] = spatial_eulerZYX_dofs();
    case 14
        [pos_dof, vel_dof] = spherical_eulerZYX_dofs();
    case 15
        [pos_dof, vel_dof] = linear_X_dofs();
    case 16
        [pos_dof, vel_dof] = linear_Y_dofs();
    case 17
        [pos_dof, vel_dof] = linear_Z_dofs();
    case 18
        [pos_dof, vel_dof] = linear_XY_dofs();
    case 19
        [pos_dof, vel_dof] = linear_XZ_dofs();
    case 20
        [pos_dof, vel_dof] = linear_YZ_dofs();
    otherwise
        error('Unrecognized Joint Type.');
    end
end