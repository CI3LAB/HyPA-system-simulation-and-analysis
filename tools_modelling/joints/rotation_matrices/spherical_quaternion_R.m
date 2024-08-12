% function that computes the rotation matrix of a spherical joint
%   input: q = [n] where n is the quaternion
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = spherical_quaternion_R(q)

    R = QuaternionToRotationMatrix(spherical_quaternion_quat(q));

end