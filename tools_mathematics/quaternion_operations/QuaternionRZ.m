% quaternion corresponds to the rotation of gamma along x axis
function n = QuaternionRZ(gamma)
    n = [cos(gamma/2); 0; 0; sin(gamma/2)];
end