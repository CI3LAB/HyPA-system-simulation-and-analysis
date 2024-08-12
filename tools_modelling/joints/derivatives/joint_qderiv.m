% function that accepts a joint type, q and v, calls the corresponding function to compute the joint position variable derivative, i.e. q_derivative
% joint type index library:
%       1 - spatial_quaternion
%       2 - spatial_eulerXYZ
%       3 - sperical_quaternion
%       4 - sperical_eulerXYZ
%       5 - universal_XY

% the inputs involve the configuration q and the velocity v. When quaternion is not involved v = q_dot, otherwise the equality does not hold.
% but in any case, the system state is comprised of both: x = [q; v]

% eulerXYZ means that from parent frame to child frame the rotations are sequentially performed along axes X, Y and Z.
function q_derivative = joint_qderiv(joint_type_index, q, v)
    switch joint_type_index
    case 1
        q_derivative = spatial_quaternion_qderiv(q, v);
    case 2
        q_derivative = spatial_eulerXYZ_qderiv(q, v);
    case 3
        q_derivative = spherical_quaternion_qderiv(q, v);
    case 4
        q_derivative = spherical_eulerXYZ_qderiv(q, v);
    case 5
        q_derivative = universal_XY_qderiv(q, v);
    case 6
        q_derivative = revolute_X_qderiv(q, v);
    case 7
        q_derivative = revolute_Y_qderiv(q, v);
    case 8
        q_derivative = revolute_Z_qderiv(q, v);
    case 9
        q_derivative = universal_XZ_qderiv(q, v);
    case 10
        q_derivative = planar_XZ_qderiv(q, v);
    case 11
        q_derivative = planar_XY_qderiv(q, v);
    case 12
        q_derivative = planar_YZ_qderiv(q, v);
    case 13
        q_derivative = spatial_eulerZYX_qderiv(q, v);
    case 14
        q_derivative = spherical_eulerZYX_qderiv(q, v);
    case 15
        q_derivative = linear_X_qderiv(q, v);
    case 16
        q_derivative = linear_Y_qderiv(q, v);
    case 17
        q_derivative = linear_Z_qderiv(q, v);
    case 18
        q_derivative = linear_XY_qderiv(q, v);
    case 19
        q_derivative = linear_XZ_qderiv(q, v);
    case 20
        q_derivative = linear_YZ_qderiv(q, v);
    otherwise
        error('Unrecognized Joint Type.');
    end
end