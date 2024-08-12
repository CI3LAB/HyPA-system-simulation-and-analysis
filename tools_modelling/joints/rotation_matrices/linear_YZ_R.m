% function that computes the rotation matrix of a linear YZ joint, which, from parent frame, moves linearly along Y and Z axis
%   input: q = [y;z] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = linear_YZ_R(~)

    R = eye(3);

end