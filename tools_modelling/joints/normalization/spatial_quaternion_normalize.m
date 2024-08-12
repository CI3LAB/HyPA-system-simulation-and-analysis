% function that normalizes the quaternion q, if applicable
%   input: q = [pos; n] where n is the quaternion and pos is the position coordinate in the PARENT FRAME
function q_normalized = spatial_quaternion_normalize(q)
    % initialize
    q_normalized = q;
    % normalize the quaternion
    q_normalized(4:7) = QuaternionNormalization(q_normalized(4:7));
end