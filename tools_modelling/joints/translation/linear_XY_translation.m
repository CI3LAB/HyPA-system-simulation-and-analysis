% function that computes the relative translation of a linear XY joint
%   input: q = [x;y] where x and y are the position coordinates in the PARENT FRAME

function xlt = linear_XY_translation(q)

    xlt = [q(1);q(2);0];

end