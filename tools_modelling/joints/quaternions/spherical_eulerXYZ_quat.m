% function that computes the Euler XYZ representation of rotation of a spherical joint
%   input: q = [alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively

function quat = spherical_eulerXYZ_quat(q)

    quat = QuaternionMultiplication(QuaternionRX(q(1)), QuaternionRY(q(2)));
    quat = QuaternionMultiplication(quat, QuaternionRZ(q(3)));

    quat = QuaternionNormalization(quat);

end