% quaternion normalization
function n_normalized = QuaternionNormalization(n)
    n_normalized = n/norm(n);
end