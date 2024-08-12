% function that computes the rotation matrix of a revolute Z joint, which, from parent frame, rotates along Z axis by \gamma
%   input: q = [\gamma] where n is the quaternion and pos is the position
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = revolute_Z_R(q)

    R = QuaternionToRotationMatrix(revolute_Z_quat(q));

end