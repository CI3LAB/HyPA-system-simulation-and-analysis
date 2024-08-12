% function that computes the quaternion representation of rotation of a revolute X joint, which, from parent frame, rotates first along X axis by \alpha
%   input: q = [\alpha] where n is the quaternion

function quat = revolute_X_quat(q)

    quat = QuaternionRX(q);

end