% function that computes the quaternion representation of rotation of a revolute Z joint, which, from parent frame, rotates first along Z axis by \gamma
%   input: q = [y; z; \alpha]

function quat = planar_YZ_quat(q)

    quat = QuaternionRX(q(3));

end