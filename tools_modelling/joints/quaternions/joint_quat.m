% function that accepts a joint type and q, calls the corresponding function to compute the quaternion representation of the relative rotation (from parent to child)
% joint type index library:
%       1 - spatial_quaternion
%       2 - spatial_eulerXYZ
%       3 - sperical_quaternion
%       4 - sperical_eulerXYZ
%       5 - universal_XY

% eulerXYZ means that from parent frame to child frame the rotations are sequentially performed along axes X, Y and Z.

function quat = joint_quat(joint_type_index, q)
    switch joint_type_index
    case 1
        quat = spatial_quaternion_quat(q);
    case 2
        quat = spatial_eulerXYZ_quat(q);
    case 3
        quat = spherical_quaternion_quat(q);
    case 4
        quat = spherical_eulerXYZ_quat(q);
    case 5
        quat = universal_XY_quat(q);
    case 6
        quat = revolute_X_quat(q);
    case 7
        quat = revolute_Y_quat(q);
    case 8
        quat = revolute_Z_quat(q);
    case 9
        quat = universal_XZ_quat(q);
    case 10
        quat = planar_XZ_quat(q);
    case 11
        quat = planar_XY_quat(q);
    case 12
        quat = planar_YZ_quat(q);
    case 13
        quat = spatial_eulerZYX_quat(q);
    case 14
        quat = spherical_eulerZYX_quat(q);
    case 15
        quat = linear_X_quat(q);
    case 16
        quat = linear_Y_quat(q);
    case 17
        quat = linear_Z_quat(q);
    case 18
        quat = linear_XY_quat(q);
    case 19
        quat = linear_XZ_quat(q);
    case 20
        quat = linear_YZ_quat(q);
    otherwise
        error('Unrecognized Joint Type.');
    end
end