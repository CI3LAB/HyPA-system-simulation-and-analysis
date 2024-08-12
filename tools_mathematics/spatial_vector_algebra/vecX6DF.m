% function that converts a 6-D spatial vector into a 6-by-6 matrix which is
% a cross product projector for a force type spatial vector
function res = vecX6DF(vec)
    vec_1to3_X3D = vecX3D(vec(1:3)); % avoid re-computing the skew symmetric matrix
    res = [ vec_1to3_X3D        vecX3D(vec(4:6));
            zeros(3)            vec_1to3_X3D];
    % or equivalently 
    % res =   -vecX6DM(vec)';
end