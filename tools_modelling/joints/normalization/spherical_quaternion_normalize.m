% function that normalizes the quaternion q, if applicable
%   input: q = [n] where n is the quaternion

function q_normalized = spherical_quaternion_normalize(q)
    % nothing special
    q_normalized = QuaternionNormalization(q);
end