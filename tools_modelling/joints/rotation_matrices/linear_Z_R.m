% function that computes the rotation matrix of a linear Z joint, which, from parent frame, moves linearly along Z axis
%   input: q = [z] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = linear_Z_R(~)

    R = eye(3);

end