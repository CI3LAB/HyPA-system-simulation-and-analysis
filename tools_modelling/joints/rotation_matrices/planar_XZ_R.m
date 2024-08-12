% function that computes the rotation matrix of a planar XZ joint, which, from parent frame, rotates along Y axis by \beta
%   input: q = [x; z; \beta] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = planar_XZ_R(q)

    R = QuaternionToRotationMatrix(planar_XZ_quat(q));

end