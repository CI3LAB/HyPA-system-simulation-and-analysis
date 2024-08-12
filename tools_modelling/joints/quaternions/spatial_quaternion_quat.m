% function that computes the quaternion representation of rotation of a spatial joint
%   input: q = [pos; n] where n is the quaternion
%                       and pos is the position coordinate in the PARENT FRAME

function quat = spatial_quaternion_quat(q)

    quat = QuaternionNormalization(q(4:7));

end