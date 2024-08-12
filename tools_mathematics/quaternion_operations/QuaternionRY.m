% quaternion corresponds to the rotation of beta along x axis
function n = QuaternionRY(beta)
    n = [cos(beta/2); 0; sin(beta/2); 0];
end