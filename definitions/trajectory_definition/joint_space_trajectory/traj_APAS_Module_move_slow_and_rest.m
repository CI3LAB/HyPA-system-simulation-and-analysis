
traj_def.robotName = 'Module';
% the name of the corresponding robot, a string

traj_def.trajectoryID = 'traj_MoveAndRest';
% the name of this trajectory, a string


traj_def.timeStep = 0.001;
% the time step between two adjacent reference trajectory points, this determines the control frequency


traj_def.numSegment = 2;
% number of trajectory segments, a positive scalar

traj_def.segment_types = cell(traj_def.numSegment,1);
% the type of the trajectory, a string array with the length of numSegment
%   'linear_interpolation'
%   'cubic_interpolation'
%   'quintic_interpolation'
%   'function'
traj_def.segment_types{1} = 'quintic_interpolation';
traj_def.segment_types{2} = 'quintic_interpolation';
% traj_def.segment_types{3} = 'quintic_interpolation';


num_dofs = 4;
traj_def.setpoints = cell(traj_def.numSegment+1,1);
% an cell array with the length of numSegment + 1
% each element contains a struct: {time, q, q_dot, q_ddot}
accumulated_time = 0.0;
motion_time = 50;
rest_time = 10;

horizontal_target = 0.06;
vertical_target = 0.0;
height_target = -1.475;
steering_angle_target = deg2rad(60.0);

idx = 1.0;
horizontal_translation_step = 0.0;
vertical_translation_step = 0.0;
height_translation_step = 0.0;
steering_angle_step = 0.0;
traj_def.setpoints{idx}.time =        accumulated_time;
traj_def.setpoints{idx}.q =           [	horizontal_translation_step;...
                                        vertical_translation_step;...
                                        height_translation_step;...
                                        steering_angle_step...
                                        ];
traj_def.setpoints{idx}.q_dot =       zeros(size(traj_def.setpoints{idx}.q));
traj_def.setpoints{idx}.q_ddot =      zeros(size(traj_def.setpoints{idx}.q));


idx = idx + 1.0;
accumulated_time = accumulated_time + motion_time;
horizontal_translation_step = horizontal_target;
vertical_translation_step = vertical_target;
height_translation_step = height_target;
steering_angle_step = steering_angle_target;
traj_def.setpoints{idx}.time =        accumulated_time;
traj_def.setpoints{idx}.q =           [	horizontal_translation_step;...
                                        vertical_translation_step;...
                                        height_translation_step;...
                                        steering_angle_step...
                                        ];
traj_def.setpoints{idx}.q_dot =       zeros(size(traj_def.setpoints{idx}.q));
traj_def.setpoints{idx}.q_ddot =      zeros(size(traj_def.setpoints{idx}.q));

idx = idx + 1.0;
accumulated_time = accumulated_time + rest_time;
traj_def.setpoints{idx}.time =        accumulated_time;
traj_def.setpoints{idx}.q =           traj_def.setpoints{idx-1}.q;
traj_def.setpoints{idx}.q_dot =       zeros(size(traj_def.setpoints{idx}.q));
traj_def.setpoints{idx}.q_ddot =      zeros(size(traj_def.setpoints{idx}.q));

traj_def.segment_functions = cell(traj_def.numSegment,1);
% a set of functions that define the corresponding trajectory segment, a cell array with the length equal to the number of segments
% only elements correspond to segments of type 'function' need to be defined
% for example, if segment i is a 'function' type segment, it shall be defined as follows