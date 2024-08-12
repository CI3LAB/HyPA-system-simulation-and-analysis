% function that converts a 3-D vector into a 3-by-3 matrix which is a cross
% product projector
function res = vecX3D(vec)
    v1 = vec(1); % less array indexing is faster
    v2 = vec(2); % less array indexing is faster
    v3 = vec(3); % less array indexing is faster
    res = [ 0   -v3 v2;
            v3  0   -v1;
            -v2 v1  0];
end