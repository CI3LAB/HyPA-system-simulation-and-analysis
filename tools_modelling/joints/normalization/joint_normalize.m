% function that accepts a joint type and q, calls the corresponding function to normalize q (this is only relevant to joints using quaternion in q)
% joint type index library:
%       1 - spatial_quaternion
%       2 - spatial_eulerXYZ
%       3 - sperical_quaternion
%       4 - sperical_eulerXYZ
%       5 - universal_XY

% the inputs involve the configuration q and the velocity v. When quaternion is not involved v = q_dot, otherwise the equality does not hold.
% but in any case, the system state is comprised of both: x = [q; v]

% eulerXYZ means that from parent frame to child frame the rotations are sequentially performed along axes X, Y and Z.
function q_normalized = joint_normalize(joint_type_index, q)
    switch joint_type_index
    case 1
        q_normalized = spatial_quaternion_normalize(q);
    case 2
        q_normalized = spatial_eulerXYZ_normalize(q);
    case 3
        q_normalized = spherical_quaternion_normalize(q);
    case 4
        q_normalized = spherical_eulerXYZ_normalize(q);
    case 5
        q_normalized = universal_XY_normalize(q);
    case 6
        q_normalized = revolute_X_normalize(q);
    case 7
        q_normalized = revolute_Y_normalize(q);
    case 8
        q_normalized = revolute_Z_normalize(q);
    case 9
        q_normalized = universal_XZ_normalize(q);
    case 10
        q_normalized = planar_XZ_normalize(q);
    case 11
        q_normalized = planar_XY_normalize(q);
    case 12
        q_normalized = planar_YZ_normalize(q);
    case 13
        q_normalized = spatial_eulerZYX_normalize(q);
    case 14
        q_normalized = spherical_eulerZYX_normalize(q);
    case 15
        q_normalized = linear_X_normalize(q);
    case 16
        q_normalized = linear_Y_normalize(q);
    case 17
        q_normalized = linear_Z_normalize(q);
    case 18
        q_normalized = linear_XY_normalize(q);
    case 19
        q_normalized = linear_XZ_normalize(q);
    case 20
        q_normalized = linear_YZ_normalize(q);
    otherwise
        error('Unrecognized Joint Type.');
    end
end