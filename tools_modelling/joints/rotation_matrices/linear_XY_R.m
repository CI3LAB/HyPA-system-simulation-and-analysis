% function that computes the rotation matrix of a linear XY joint, which, from parent frame, moves linearly along X and Y axis
%   input: q = [x;y] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = linear_XY_R(~)

    R = eye(3);

end