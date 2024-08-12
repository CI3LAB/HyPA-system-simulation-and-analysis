% function that computes the relative translation of a linear YZ joint
%   input: q = [y;z] where y and z are the position coordinates in the PARENT FRAME

function xlt = linear_YZ_translation(q)

    xlt = [0;q(1);q(2)];

end