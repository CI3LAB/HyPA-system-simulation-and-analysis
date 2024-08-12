% quaternion to rotation matrix, x' = R x where x' is the coordinate in the moving frame, x is the coordinate in the fixed frame
function R = QuaternionToRotationMatrix(n)
    
    % approach 1
    % eta = n(1);
    % q = n(2:4);
    % R = (eta^2-q'*q)*eye(3,3) + 2*(q*q') - 2*eta*vecX3D(q);

    % approach 2
    R = QuaternionMatrixG(n)*(QuaternionMatrixE(n))';
end