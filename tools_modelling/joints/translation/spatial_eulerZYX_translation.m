% function that computes the relative translation of a spatial joint
%   input: q = [pos; alpha; beta; gamma] where alpha, beta and gamma are the Euler angles along X, Y and Z axes, respectively
%                                        and pos is the position coordinate in the PARENT FRAME

function xlt = spatial_eulerZYX_translation(q)

    xlt = q(1:3);

end