% function that computes the quaternion representation of rotation of a revolute Z joint, which, from parent frame, rotates first along Z axis by \gamma
%   input: q = [x; y; \gamma]

function quat = planar_XY_quat(q)

    quat = QuaternionRZ(q(3));

end