% function that computes the Euler XYZ representation of rotation of a spatial joint
%   input: q = [pos; alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%                                        and pos is the position coordinate in the PARENT FRAME

function quat = spatial_eulerZYX_quat(q)

    quat = QuaternionMultiplication(QuaternionRZ(q(6)), QuaternionRY(q(5)));
    quat = QuaternionMultiplication(quat, QuaternionRX(q(4)));

    quat = QuaternionNormalization(quat);

end