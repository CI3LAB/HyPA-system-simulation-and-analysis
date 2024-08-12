% function that computes the rotation matrix of a planar YZ joint, which, from parent frame, rotates along X axis by \alpha
%   input: q = [y; z; \alpha] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = planar_YZ_R(q)

    R = QuaternionToRotationMatrix(planar_YZ_quat(q));

end