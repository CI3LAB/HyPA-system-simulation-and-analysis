% function that computes the quaternion representation of rotation of a revolute Y joint, which, from parent frame, rotates first along Y axis by \beta
%   input: q = [\beta] where n is the quaternion

function quat = revolute_Y_quat(q)

    quat = QuaternionRY(q);

end