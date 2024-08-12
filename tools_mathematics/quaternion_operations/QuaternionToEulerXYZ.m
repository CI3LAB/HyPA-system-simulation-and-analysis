function [alpha, beta, gamma] = QuaternionToEulerXYZ(n)
    % the input quaternion represents the orientation of the moving frame
    % w.r.t. the base frame while the input quaternion to quat2eul needs to
    % be a quaternion representing the orientation of the moving frame 
    % w.r.t. the base frame as a row vector
    % additionally, the rotation sequence 'XYZ' represents the rotation
    % sequence from the base frame to the moving frame
    eul = quat2eul(n','XYZ');
    alpha = eul(1);
    beta = eul(2);
    gamma = eul(3);
end