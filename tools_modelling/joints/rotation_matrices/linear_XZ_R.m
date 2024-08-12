% function that computes the rotation matrix of a linear XZ joint, which, from parent frame, moves linearly along X and Z axis
%   input: q = [x;z] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = linear_XZ_R(~)

    R = eye(3);

end