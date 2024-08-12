% quaternion corresponds to the rotation of alpha along x axis
function n = QuaternionRX(alpha)
    n = [cos(alpha/2); sin(alpha/2); 0; 0];
end