% function that normalizes the quaternion q, if applicable
%   input: q = [pos; alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%                                        and pos is the position coordinate in the PARENT FRAME
function q_normalized = spatial_eulerZYX_normalize(q)
    % nothing special
    q_normalized = q;
end