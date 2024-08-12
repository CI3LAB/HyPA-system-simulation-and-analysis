% multiply two quaternions to get a new one to represent the total
% orientation
function n_res = QuaternionMultiplication(n_l, n_r)
    % if we make sure that in every quaternion generating function, the generated quaternion is normalized, we won't need to do the normalization here
    % n_l = QuaternionNormalization(n_l);
    % n_r = QuaternionNormalization(n_r);
    % n_res = n_l circle n_r

    % method 1: slower but easier to read
    % n_res = zeros(4,1);
    % n_res(1) = n_l(1)*n_r(1) - n_l(2:4)'*n_r(2:4);
    % n_res(2:4) = n_l(1)*n_r(2:4) + n_r(1)*n_l(2:4) + vecX3D(n_l(2:4))*n_r(2:4);

    % method 2: faster but harder to read (less array indexing is faster)
    n_l_s = n_l(1);
    n_l_v = [n_l(2);n_l(3);n_l(4)];
    n_r_s = n_r(1);
    n_r_v = [n_r(2);n_r(3);n_r(4)];
%     n_res = [n_l_s*n_r_s - n_l_v'*n_r_v; n_l_s*n_r_v + n_r_s*n_l_v + vecX3D(n_l_v)*n_r_v];
    n_res = [n_l_s*n_r_s - n_l_v'*n_r_v; n_l_s*n_r_v + n_r_s*n_l_v + cross(n_l_v, n_r_v)]; % directly do cross product is faster
    
%     n_res = QuaternionNormalization(n_res);
end