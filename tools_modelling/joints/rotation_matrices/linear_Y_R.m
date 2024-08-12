% function that computes the rotation matrix of a linear Y joint, which, from parent frame, moves linearly along Y axis
%   input: q = [y] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = linear_Y_R(~)

    R = eye(3);

end