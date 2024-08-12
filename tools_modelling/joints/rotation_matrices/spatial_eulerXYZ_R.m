% function that computes the rotation matrix of a spatial joint
%   input: q = [pos; alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%                                        and pos is the position coordinate in the PARENT FRAME
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = spatial_eulerXYZ_R(q)

    R = QuaternionToRotationMatrix(spatial_eulerXYZ_quat(q));

end