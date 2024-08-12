% function that computes the quaternion representation of rotation of a revolute Z joint, which, from parent frame, rotates first along Z axis by \gamma
%   input: q = [\gamma] where n is the quaternion

function quat = revolute_Z_quat(q)

    quat = QuaternionRZ(q);

end