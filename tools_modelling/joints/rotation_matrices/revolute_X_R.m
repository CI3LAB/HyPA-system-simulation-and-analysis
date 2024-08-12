% function that computes the rotation matrix of a revolute X joint, which, from parent frame, rotates along X axis by \alpha
%   input: q = [\alpha] where n is the quaternion and pos is the position
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = revolute_X_R(q)

    R = QuaternionToRotationMatrix(revolute_X_quat(q));

end