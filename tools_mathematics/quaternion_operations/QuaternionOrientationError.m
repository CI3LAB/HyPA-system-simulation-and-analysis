% derive the quaternion orientation error which represents a rotation direction following which the orientation can be rotated from n_2 to n_1:
% n_1 = n_2 \circ n_delta, or analogously n_delta = n_1 - n_2, where n_delta = [eta; eo]
% One additional remark is that the frame where e0 is represented in is not essential here since the vectors are the same (either represented in frame 1 or frame 2)
function eo = QuaternionOrientationError(n_1, n_2)
    % n_1 is a quaternion representing the actual orientation
    % n_2 is a quaternion representing the desired orientation
    % eo is a 3-d vector represeting the orientation error, it is
    % also a direction of angular velocity that can drive n_1 to
    % n_2
    % n_1 = QuaternionNormalization(n_1);
    % n_2 = QuaternionNormalization(n_2);
    n_1_v = [n_1(2);n_1(3);n_1(4)]; % less array indexing is faster
    n_2_v = [n_2(2);n_2(3);n_2(4)]; % less array indexing is faster
    % eo = n_2(1)*n_1_v - n_1(1)*n_2_v - vecX3D(n_2_v)*n_1_v;
    eo = n_2(1)*n_1_v - n_1(1)*n_2_v - cross(n_2_v, n_1_v); % directly do cross product is faster
end