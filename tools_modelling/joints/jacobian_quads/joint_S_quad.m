% this function computes a matrix S_quad which satisfies: v_dot = q_dot'*S_quad*q_dot + S*q_ddot, i.e. S_quad = dS/dq_dot
function [S_quad] = joint_S_quad(joint_type_index, q)
    switch joint_type_index
    case 1
        [S_quad] = spatial_quaternion_S_quad(q);
    case 2
        [S_quad] = spatial_eulerXYZ_S_quad(q);
    case 3
        [S_quad] = spherical_quaternion_S_quad(q);
    case 4
        [S_quad] = spherical_eulerXYZ_S_quad(q);
    case 5
        [S_quad] = universal_XY_S_quad(q);
    case 6
        [S_quad] = revolute_X_S_quad(q);
    case 7
        [S_quad] = revolute_Y_S_quad(q);
    case 8
        [S_quad] = revolute_Z_S_quad(q);
    case 9
        [S_quad] = universal_XZ_S_quad(q);
    case 10
        [S_quad] = planar_XZ_S_quad(q);
    case 11
        [S_quad] = planar_XY_S_quad(q);
    case 12
        [S_quad] = planar_YZ_S_quad(q);
    case 13
        [S_quad] = spatial_eulerZYX_S_quad(q);
    case 14
        [S_quad] = spherical_eulerZYX_S_quad(q);
    case 15
        [S_quad] = linear_X_S_quad(q);
    case 16
        [S_quad] = linear_Y_S_quad(q);
    case 17
        [S_quad] = linear_Z_S_quad(q);
    case 18
        [S_quad] = linear_XY_S_quad(q);
    case 19
        [S_quad] = linear_XZ_S_quad(q);
    case 20
        [S_quad] = linear_YZ_S_quad(q);
    otherwise
        error('Unrecognized Joint Type.');
    end
end