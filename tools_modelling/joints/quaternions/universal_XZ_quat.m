% function that computes the quaternion representation of rotation of a universal XZ joint, which, from parent frame, rotates first along X axis by \alpha, then rotates along the local Y axis by \beta
%   input: q = [\alpha; \gamma] where n is the quaternion

function quat = universal_XZ_quat(q)

    quat = QuaternionMultiplication(QuaternionRX(q(1)), QuaternionRZ(q(2)));

    quat = QuaternionNormalization(quat);

end