
traj_def.robotName = [];
    % the name of the corresponding robot, a string

traj_def.trajectoryID = [];
    % the name of this trajectory, a string

traj_def.timeStep = [];
    % the time step between two adjacent reference trajectory points, this determines the control frequency

traj_def.numSegment = [];
    % number of trajectory segments, a positive scalar

traj_def.segment_type = cell(traj_def.numSegment,1);
    % the type of the trajectory, a string array with the length of numSegment
    %   'linear_interpolation'
    %   'cubic_interpolation'
    %   'quintic_interpolation'
    %   'function'

traj_def.setpoints = cell(traj_def.numSegment+1,1);
    % an cell array with the length of numSegment + 1
    % each element contains a struct: {time, q, q_dot, q_ddot}

traj_def.segment_functions = cell(traj_def.numSegment,1);
    % a set of functions that define the corresponding trajectory segment, a cell array with the length equal to the number of segments
    % only elements correspond to segments of type 'function' need to be defined
    % for example, if segment i is a 'function' type segment, it shall be defined as follows
    traj_def.segment_functions{i} = @(t) func_segment_i(t);
    % where func_segment_i is defined within the file:
    function [q, q_dot, q_ddot] = func_segment_i(t)
    end