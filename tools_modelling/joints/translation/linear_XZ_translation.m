% function that computes the relative translation of a linear XZ joint
%   input: q = [x;z] where x and z are the position coordinates in the PARENT FRAME

function xlt = linear_XZ_translation(q)

    xlt = [q(1);0;q(2)];

end