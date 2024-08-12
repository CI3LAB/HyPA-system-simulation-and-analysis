% function that computes the relative translation of a linear Y joint
%   input: q = [y] where y is the position coordinates in the PARENT FRAME

function xlt = linear_Y_translation(q)

    xlt = [0;q(1);0];

end