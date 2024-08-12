% function that accepts a joint type and q, calls the corresponding function to compute the relative translation (from parent to child IN PARENT FRAME)
% joint type index library:
%       1 - spatial_quaternion
%       2 - spatial_eulerXYZ
%       3 - sperical_quaternion
%       4 - sperical_eulerXYZ
%       5 - universal_XY

% eulerXYZ means that from parent frame to child frame the rotations are sequentially performed along axes X, Y and Z.

function xlt = joint_translation(joint_type_index, q)
    switch joint_type_index
    case 1
        xlt = spatial_quaternion_translation(q);
    case 2
        xlt = spatial_eulerXYZ_translation(q);
    case 3
        xlt = spherical_quaternion_translation(q);
    case 4
        xlt = spherical_eulerXYZ_translation(q);
    case 5
        xlt = universal_XY_translation(q);
    case 6
        xlt = revolute_X_translation(q);
    case 7
        xlt = revolute_Y_translation(q);
    case 8
        xlt = revolute_Z_translation(q);
    case 9
        xlt = universal_XZ_translation(q);
    case 10
        xlt = planar_XZ_translation(q);
    case 11
        xlt = planar_XY_translation(q);
    case 12
        xlt = planar_YZ_translation(q);
    case 13
        xlt = spatial_eulerZYX_translation(q);
    case 14
        xlt = spherical_eulerZYX_translation(q);
    case 15
        xlt = linear_X_translation(q);
    case 16
        xlt = linear_Y_translation(q);
    case 17
        xlt = linear_Z_translation(q);
    case 18
        xlt = linear_XY_translation(q);
    case 19
        xlt = linear_XZ_translation(q);
    case 20
        xlt = linear_YZ_translation(q);
    otherwise
        error('Unrecognized Joint Type.');
    end
end