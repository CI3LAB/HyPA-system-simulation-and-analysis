% function that rotates a given vector x_vec using the given quaternion n: x' = \bar{n} \circ x \circ n where x is the expanded x_vec, i.e. x = [0; x_vec];

function x_vec_res = QuaternionVectorRotation(n, x_vec)

    if norm(x_vec) < 1e-12
        x_vec_res = x_vec;
    else

        x = [0; x_vec(1); x_vec(2); x_vec(3)];

        x_res = QuaternionMultiplication(QuaternionConjugate(n), x);
        x_res = QuaternionMultiplication(x_res, n);

        x_vec_res = x_res(2:4);
    end

end