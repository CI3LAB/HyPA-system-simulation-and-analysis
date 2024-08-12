% function that computes the quaternion representation of rotation of a revolute Z joint, which, from parent frame, rotates first along Z axis by \gamma
%   input: q = [x; z; \beta]

function quat = planar_XZ_quat(q)

    quat = QuaternionRY(q(3));

end