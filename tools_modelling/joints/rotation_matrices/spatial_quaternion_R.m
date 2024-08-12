% function that computes the rotation matrix of a spatial joint
%   input: q = [pos; n] where n is the quaternion
%                       and pos is the position coordinate in the PARENT FRAME
%   output: R, x' = R x where x' is the coordinate in the moving frame. in other words, R converts a vector in the vector frame into the child (rotating) frame

function R = spatial_quaternion_R(q)

    R = QuaternionToRotationMatrix(spatial_quaternion_quat(q));

end