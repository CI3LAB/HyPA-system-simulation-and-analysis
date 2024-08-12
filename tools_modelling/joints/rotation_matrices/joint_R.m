% function that accepts a joint type and q, calls the corresponding function to compute the relative rotation matrix (from parent to child)
% joint type index library:
%       1 - spatial_quaternion
%       2 - spatial_eulerXYZ
%       3 - sperical_quaternion
%       4 - sperical_eulerXYZ
%       5 - universal_XY

% eulerXYZ means that from parent frame to child frame the rotations are sequentially performed along axes X, Y and Z.
function R = joint_R(joint_type_index, q)
    switch joint_type_index
    case 1
        R = spatial_quaternion_R(q);
    case 2
        R = spatial_eulerXYZ_R(q);
    case 3
        R = spherical_quaternion_R(q);
    case 4
        R = spherical_eulerXYZ_R(q);
    case 5
        R = universal_XY_R(q);
    case 6
        R = revolute_X_R(q);
    case 7
        R = revolute_Y_R(q);
    case 8
        R = revolute_Z_R(q);
    case 9
        R = universal_XZ_R(q);
    case 10
        R = planar_XZ_R(q);
    case 11
        R = planar_XY_R(q);
    case 12
        R = planar_YZ_R(q);
    case 13
        R = spatial_eulerZYX_R(q);
    case 14
        R = spherical_eulerZYX_R(q);
    case 15
        R = linear_X_R(q);
    case 16
        R = linear_Y_R(q);
    case 17
        R = linear_Z_R(q);
    case 18
        R = linear_XY_R(q);
    case 19
        R = linear_XZ_R(q);
    case 20
        R = linear_YZ_R(q);
    otherwise
        error('Unrecognized Joint Type.');
    end
end