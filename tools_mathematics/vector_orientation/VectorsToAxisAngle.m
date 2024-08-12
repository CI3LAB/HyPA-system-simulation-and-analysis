function [theta, axis] = VectorsToAxisAngle(unit_vec_start, unit_vec_end)
    % the range of theta is [0, pi]
    axis = cross(unit_vec_start, unit_vec_end);
    theta = acos(dot(unit_vec_start, unit_vec_end));
    
end