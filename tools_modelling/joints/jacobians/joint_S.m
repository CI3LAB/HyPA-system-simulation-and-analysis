% function that accepts a joint type and q, calls the corresponding function to compute the joint Jacobian matrix which maps the joint velocity to body velocity
% the body velocity is assumed to be defined in the local frame with angular velocity comes first, followed by the linear velocity
% joint type index library:
%       1 - spatial_quaternion
%       2 - spatial_eulerXYZ
%       3 - sperical_quaternion
%       4 - sperical_eulerXYZ
%       5 - universal_XY

% the inputs involve the configuration q and the velocity v. When quaternion is not involved v = q_dot, otherwise the equality does not hold.
% but in any case, the system state is comprised of both: x = [q; v]

% eulerXYZ means that from parent frame to child frame the rotations are sequentially performed along axes X, Y and Z.
function [S, S_deriv] = joint_S(joint_type_index, q, v)
    switch joint_type_index
    case 1
        [S, S_deriv] = spatial_quaternion_S(q, v);
    case 2
        [S, S_deriv] = spatial_eulerXYZ_S(q, v);
    case 3
        [S, S_deriv] = spherical_quaternion_S(q, v);
    case 4
        [S, S_deriv] = spherical_eulerXYZ_S(q, v);
    case 5
        [S, S_deriv] = universal_XY_S(q, v);
    case 6
        [S, S_deriv] = revolute_X_S(q, v);
    case 7
        [S, S_deriv] = revolute_Y_S(q, v);
    case 8
        [S, S_deriv] = revolute_Z_S(q, v);
    case 9
        [S, S_deriv] = universal_XZ_S(q, v);
    case 10
        [S, S_deriv] = planar_XZ_S(q, v);
    case 11
        [S, S_deriv] = planar_XY_S(q, v);
    case 12
        [S, S_deriv] = planar_YZ_S(q, v);
    case 13
        [S, S_deriv] = spatial_eulerZYX_S(q, v);
    case 14
        [S, S_deriv] = spherical_eulerZYX_S(q, v);
    case 15
        [S, S_deriv] = linear_X_S(q, v);
    case 16
        [S, S_deriv] = linear_Y_S(q, v);
    case 17
        [S, S_deriv] = linear_Z_S(q, v);
    case 18
        [S, S_deriv] = linear_XY_S(q, v);
    case 19
        [S, S_deriv] = linear_XZ_S(q, v);
    case 20
        [S, S_deriv] = linear_YZ_S(q, v);
    otherwise
        error('Unrecognized Joint Type.');
    end
end