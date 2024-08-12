% function computes the angle (in radians) between vec1 and vec2.
% pos_direction is a vector outside vec1-vec2 plane that represents the position rotation direction
% result is ranged as [-pi, pi]
function theta = VectorAngle2(vec1, vec2, pos_direction)
    x = cross(vec1,vec2);
    c = sign(dot(x,pos_direction))*norm(x);
    theta = atan2(c,dot(vec1,vec2));
    % theta = atan(c/dot(vec1,vec2));
end