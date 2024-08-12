% function that computes the rotation matrix of a spherical joint
%   input: q = [alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = spherical_eulerXYZ_R(q)

    R = QuaternionToRotationMatrix(spherical_eulerXYZ_quat(q));

end