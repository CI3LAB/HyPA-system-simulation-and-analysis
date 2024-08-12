% function that computes the Euler XYZ representation of rotation of a spatial joint
%   input: q = [pos; alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%                                        and pos is the position coordinate in the PARENT FRAME

function quat = spatial_eulerXYZ_quat(q)

    quat = QuaternionMultiplication(QuaternionRX(q(4)), QuaternionRY(q(5)));
    quat = QuaternionMultiplication(quat, QuaternionRZ(q(6)));

    quat = QuaternionNormalization(quat);

end