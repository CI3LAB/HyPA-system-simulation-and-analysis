% function that computes the relative translation of a spatial joint
%   input: q = [x; z; \beta] where \beta is the Euler angle along Y axis
%                                        and x and z are the position coordinates in the PARENT FRAME

function xlt = planar_XZ_translation(q)

    xlt = [q(1); 0; q(2)];

end