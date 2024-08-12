% the function measures the minimum rotation between two vectors in the form of EulerXYZ angles (rotation starts from vec_start and ends up with vec_end)
function [alpha, beta, gamma] = VectorsToEulerXYZ(vec_start, vec_end)

    [phi, axis] = VectorsToAxisAngle(vec_start, vec_end);
    n = AxisAngleToQuaternion(phi, axis);
    [alpha, beta, gamma] = QuaternionToEulerXYZ(n);

end