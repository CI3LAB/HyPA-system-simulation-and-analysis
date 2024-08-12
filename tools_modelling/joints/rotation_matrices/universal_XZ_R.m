% function that computes the rotation matrix of a universal XZ joint, which, from parent frame, rotates first along X axis by \alpha, then rotates along the local Z axis by \gamma
%   input: q = [\alpha; \gamma] where n is the quaternion and pos is the position
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = universal_XZ_R(q)

    R = QuaternionToRotationMatrix(universal_XZ_quat(q));

end