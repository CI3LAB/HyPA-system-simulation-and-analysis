% angular velocity to subsequent quaternion
function n_next = QuaternionDiscreteUpdate(n, w, T)
    % assume w is the angular velocity in the rotating frame
    A = 0.5*[   0       -w'
                w       -vecX3D(w)];
    % % assume w is the angular velocity in the stationary frame
    % A = 0.5*[   0       -w'
    %             w       vecX3D(w)];

    n_next = expm(A*T)*n;
    n_next = QuaternionNormalization(n_next);
end