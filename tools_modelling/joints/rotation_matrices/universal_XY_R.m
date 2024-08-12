% function that computes the rotation matrix of a universal XY joint, which, from parent frame, rotates first along X axis by \alpha, then rotates along the local Y axis by \beta
%   input: q = [\alpha; \beta] where n is the quaternion and pos is the position
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = universal_XY_R(q)

    R = QuaternionToRotationMatrix(universal_XY_quat(q));

end