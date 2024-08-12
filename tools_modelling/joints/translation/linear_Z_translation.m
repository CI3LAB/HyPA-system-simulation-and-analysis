% function that computes the relative translation of a linear Z joint
%   input: q = [z] where z is the position coordinates in the PARENT FRAME

function xlt = linear_Z_translation(q)

    xlt = [0;0;q(1)];

end