% function that computes the rotation matrix of a linear X joint, which, from parent frame, moves linearly along X axis
%   input: q = [x] 
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = linear_X_R(~)

    R = eye(3);

end