% angular velocity to quaternion derivative
function n_dot = QuaternionDerivative(n, w)
    % assume w is the angular velocity in the rotating frame
    A = 0.5*[   0       -w'
                w       -vecX3D(w)];
    % % assume w is the angular velocity in the stationary frame
    % A = 0.5*[   0       -w'
    %             w       vecX3D(w)];

    n_dot = A*n;
end