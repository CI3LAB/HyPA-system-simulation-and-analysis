% function that computes the quaternion representation of rotation of a spherical joint
%   input: q = [n] where n is the quaternion

function quat = spherical_quaternion_quat(q)

    quat = QuaternionNormalization(q(1:4));

end