% function that computes the relative translation of a spatial joint
%   input: q = [x; y; \gamma] where \gamma is the Euler angle along Z axis
%                                        and x and y are the position coordinates in the PARENT FRAME

function xlt = planar_XY_translation(q)

    xlt = [q(1);q(2); 0];

end