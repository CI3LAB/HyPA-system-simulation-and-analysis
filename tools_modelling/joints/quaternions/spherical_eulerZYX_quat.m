% function that computes the Euler XYZ representation of rotation of a spherical joint
%   input: q = [alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively

function quat = spherical_eulerZYX_quat(q)

    quat = QuaternionMultiplication(QuaternionRZ(q(3)), QuaternionRY(q(2)));
    quat = QuaternionMultiplication(quat, QuaternionRX(q(1)));

    quat = QuaternionNormalization(quat);

end