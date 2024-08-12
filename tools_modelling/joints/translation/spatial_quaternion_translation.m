% function that computes the relative translation of rotation of a spatial joint
%   input: q = [pos; n] where n is the quaternion
%                       and pos is the position coordinate in the PARENT FRAME

function xlt = spatial_quaternion_translation(q)

    xlt = q(1:3);

end