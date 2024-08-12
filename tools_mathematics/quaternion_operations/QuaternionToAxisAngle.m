function [theta, axis] = QuaternionToAxisAngle(n)
    % here theta should be confined within 2*[-pi/2, pi/2]
    n = n/norm(n);
    n_v = n(2:4);
    norm_tmp = norm(n_v);
    theta = 2*atan(norm_tmp/n(1));
    if norm_tmp < 1e-8
        axis = zeros(3,1);
    else
        axis = n_v/norm_tmp;
    end
end