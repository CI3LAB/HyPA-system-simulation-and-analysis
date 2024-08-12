% function that computes the relative translation of a spatial joint
%   input: q = [y; z; \alpha] where \alpha is the Euler angle along X axis
%                                        and y and z are the position coordinates in the PARENT FRAME

function xlt = planar_YZ_translation(q)

    xlt = [0; q(1); q(2)];

end