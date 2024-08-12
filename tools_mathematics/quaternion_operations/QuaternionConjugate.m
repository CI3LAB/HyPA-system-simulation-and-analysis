% function that gives the conjugate of a given quaternion

function n_bar = QuaternionConjugate(n)
    n_bar = -n;
    n_bar(1) = -n_bar(1);
end