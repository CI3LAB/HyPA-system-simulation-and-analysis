% function that computes the rotation matrix of a planar XY joint, which, from parent frame, rotates along Z axis by \gamma
%   input: q = [x; y; \gamma] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = planar_XY_R(q)

    R = QuaternionToRotationMatrix(planar_XY_quat(q));

end