clc; clear; close all;
%% build the model
def_apas_var0;
% def_apas_realistic;
mdl = RigidBodySystem(model_def);

%% create the trajectory
% traj_APAS_Module_rest;
% traj_APAS_Module_move_and_rest;
% traj_APAS_Module_steer_and_rest;
% traj_APAS_Module_move_slow_and_rest;
traj_APAS_Module_move_slow_and_rest_long;
refTrajObj = JointSpaceTrajectory(traj_def);

%% create controllers

bypass_stabilization_step = false;

len_error_scalar_crane_cable = 80;
len_error_scalar_hoisting_cable = 80;
len_error_scalar_module_cable = 80;
vel_error_scalar_crane_cable = 80;
vel_error_scalar_hoisting_cable = 80;
vel_error_scalar_module_cable = 80;
damping_ratio_crane_cable = 6;
damping_ratio_hoisting_cable = 6;
damping_ratio_module_cable = 6;
load_factor_crane_cable = 10;
load_factor_hoisting_cable = 10;
load_factor_module_cable = 10;
damping_ratio_steering_motor = 100;
joint_feedback_weight = 0.000;
steering_motor_torque_lpf = 1.0;
damping_ratio_linear_motor = 10;

thruster_vel_threshold = 0.0;
thruster_gain = 200.0;
% thruster_gain = 50.0;
% thruster_gain = 0000.0;
thruster_force_max = 50.0;
thruster_force_min = -0.0;

fmin = 0.0;

if strcmp(mdl.robotName, 'APAS_var0')
    crane_cable_load = 10 * sum(mdl.m(:)) / 2.0
    hoisting_cable_load = 10 * sum(mdl.m(4:end)) / 4.0
    module_cable_load = 10 * sum(mdl.m(end)) / 4.0
elseif strcmp(mdl.robotName, 'APASR_var0')
    crane_cable_load = 10 * sum(mdl.m(:)) / 2.0
    hoisting_cable_load = 10 * sum(mdl.m(17:end)) / 4.0
    module_cable_load = 10 * sum(mdl.m(end)) / 4.0
end

positionLoopPower = 1.0;
velocityLoopPower = 1.0;
crane_cable_u_max = crane_cable_load * load_factor_crane_cable;
% crane_cable_ep_max = 0.01;
crane_cable_ep_max = 0.001;
crane_cable_ev_max = 0.1;
crane_cable_kp = crane_cable_load / crane_cable_ep_max;
crane_cable_kd = 5e4;
crane_cable_kd = damping_ratio_crane_cable * sqrt(crane_cable_kp);
% crane_cable_u_max = 5e4;
% crane_cable_ep_max = 10;
% crane_cable_ev_max = 10;
% crane_cable_kp = 1e6;
% crane_cable_kd = 5e3;
crane_cable_cpid(1) = PID(crane_cable_u_max, crane_cable_ep_max * len_error_scalar_crane_cable, crane_cable_ev_max * vel_error_scalar_crane_cable, crane_cable_kp, crane_cable_kd);
crane_cable_cpid(2) = PID(crane_cable_u_max, crane_cable_ep_max * len_error_scalar_crane_cable, crane_cable_ev_max * vel_error_scalar_crane_cable, crane_cable_kp, crane_cable_kd);
crane_cable_cpid(1).setUmin(fmin);
crane_cable_cpid(2).setUmin(fmin);
crane_cable_cpid(1).setGainCurve(positionLoopPower, velocityLoopPower);
crane_cable_cpid(2).setGainCurve(positionLoopPower, velocityLoopPower);
crane_cable_cpid(1).setCableLengthController();
crane_cable_cpid(2).setCableLengthController();

positionLoopPower = 1.0; % make this larger than 1 (1.3) so that when cable is close to slack the cable force profile is smoother (oscillates with smaller amplitidue)
velocityLoopPower = 1.0;
hoisting_cable_u_max = hoisting_cable_load * load_factor_hoisting_cable;
hoisting_cable_ep_max = 0.01;
hoisting_cable_ev_max = 0.1;
hoisting_cable_kp = hoisting_cable_load / hoisting_cable_ep_max;
hoisting_cable_kd = 1.0e2;
hoisting_cable_kd = damping_ratio_hoisting_cable * sqrt(hoisting_cable_kp);
% hoisting_cable_u_max = 5e4;
% hoisting_cable_ep_max = 10;
% hoisting_cable_ev_max = 10;
% hoisting_cable_kp = 2e5;
% hoisting_cable_kd = 1.0e3;
hoisting_cable_cpid(1) = PID(hoisting_cable_u_max, hoisting_cable_ep_max * len_error_scalar_hoisting_cable, hoisting_cable_ev_max * vel_error_scalar_hoisting_cable, hoisting_cable_kp, hoisting_cable_kd);
hoisting_cable_cpid(2) = PID(hoisting_cable_u_max, hoisting_cable_ep_max * len_error_scalar_hoisting_cable, hoisting_cable_ev_max * vel_error_scalar_hoisting_cable, hoisting_cable_kp, hoisting_cable_kd);
hoisting_cable_cpid(3) = PID(hoisting_cable_u_max, hoisting_cable_ep_max * len_error_scalar_hoisting_cable, hoisting_cable_ev_max * vel_error_scalar_hoisting_cable, hoisting_cable_kp, hoisting_cable_kd);
hoisting_cable_cpid(4) = PID(hoisting_cable_u_max, hoisting_cable_ep_max * len_error_scalar_hoisting_cable, hoisting_cable_ev_max * vel_error_scalar_hoisting_cable, hoisting_cable_kp, hoisting_cable_kd);
hoisting_cable_cpid(1).setUmin(fmin);
hoisting_cable_cpid(2).setUmin(fmin);
hoisting_cable_cpid(3).setUmin(fmin);
hoisting_cable_cpid(4).setUmin(fmin);
hoisting_cable_cpid(1).setGainCurve(positionLoopPower, velocityLoopPower);
hoisting_cable_cpid(2).setGainCurve(positionLoopPower, velocityLoopPower);
hoisting_cable_cpid(3).setGainCurve(positionLoopPower, velocityLoopPower);
hoisting_cable_cpid(4).setGainCurve(positionLoopPower, velocityLoopPower);
hoisting_cable_cpid(1).setCableLengthController();
hoisting_cable_cpid(2).setCableLengthController();
hoisting_cable_cpid(3).setCableLengthController();
hoisting_cable_cpid(4).setCableLengthController();

positionLoopPower = 1.0;
velocityLoopPower = 1.0;
module_cable_u_max = module_cable_load * load_factor_module_cable;
% module_cable_ep_max = 0.01;
module_cable_ep_max = 0.001;
module_cable_ev_max = 0.1;
module_cable_kp = module_cable_load / module_cable_ep_max;
module_cable_kd = 1e4;
module_cable_kd = damping_ratio_module_cable * sqrt(module_cable_kp);
% module_cable_u_max = 5e4;
% module_cable_ep_max = 10;
% module_cable_ev_max = 10;
% module_cable_kp = 1e6;
% module_cable_kd = 5e3;
module_cable_cpid(1) = PID(module_cable_u_max, module_cable_ep_max * len_error_scalar_module_cable, module_cable_ev_max * vel_error_scalar_module_cable, module_cable_kp, module_cable_kd);
module_cable_cpid(2) = PID(module_cable_u_max, module_cable_ep_max * len_error_scalar_module_cable, module_cable_ev_max * vel_error_scalar_module_cable, module_cable_kp, module_cable_kd);
module_cable_cpid(3) = PID(module_cable_u_max, module_cable_ep_max * len_error_scalar_module_cable, module_cable_ev_max * vel_error_scalar_module_cable, module_cable_kp, module_cable_kd);
module_cable_cpid(4) = PID(module_cable_u_max, module_cable_ep_max * len_error_scalar_module_cable, module_cable_ev_max * vel_error_scalar_module_cable, module_cable_kp, module_cable_kd);
module_cable_cpid(1).setUmin(fmin);
module_cable_cpid(2).setUmin(fmin);
module_cable_cpid(3).setUmin(fmin);
module_cable_cpid(4).setUmin(fmin);
module_cable_cpid(1).setGainCurve(positionLoopPower, velocityLoopPower);
module_cable_cpid(2).setGainCurve(positionLoopPower, velocityLoopPower);
module_cable_cpid(3).setGainCurve(positionLoopPower, velocityLoopPower);
module_cable_cpid(4).setGainCurve(positionLoopPower, velocityLoopPower);
module_cable_cpid(1).setCableLengthController();
module_cable_cpid(2).setCableLengthController();
module_cable_cpid(3).setCableLengthController();
module_cable_cpid(4).setCableLengthController();

steering_inertia_gain = 1;
steering_u_max = 3e3;
steering_ep_max = 0.1;
steering_ev_max = 100.0;
steering_kp = steering_u_max / steering_ep_max;
steering_kd = steering_inertia_gain * damping_ratio_steering_motor * sqrt(steering_kp / steering_inertia_gain);
steering_motor_cpid = PID(steering_u_max + steering_kd * steering_ev_max, steering_ep_max, steering_ev_max, steering_kp, steering_kd);
% steering_motor_cpid.setCableLengthController();

linear_u_max = 2e4;
linear_ep_max = 0.01;
linear_ev_max = 0.5;
linear_kp = linear_u_max / linear_ep_max;
linear_kd = damping_ratio_linear_motor * sqrt(linear_kp);
linear_motor_cpid(1) = PID(linear_u_max, linear_ep_max, linear_ev_max, linear_kp, linear_kd);
linear_motor_cpid(2) = PID(linear_u_max, linear_ep_max, linear_ev_max, linear_kp, linear_kd);
% linear_motor_cpid(1).setCableLengthController();
% linear_motor_cpid(2).setCableLengthController();

crane_cable_ff = zeros(2, 1);
hoisting_cable_ff = zeros(4, 1);
module_cable_ff = zeros(4, 1);
% [crane_cable_ff, hoisting_cable_ff, module_cable_ff] = APASCalculateFeedforwardInput(mdl);
crane_cable_ff = 0.0 * crane_cable_ff;
hoisting_cable_ff = 0.0 * hoisting_cable_ff;
module_cable_ff = 0.0 * module_cable_ff;

%% setup simulation

plot_axis = [0.0 2.0 0.0 2.0 2.0 10.5];
plot_axis = [-0.2 10.2 -0.2 10.2 -0.1 60.1];
view_angle = [45 0];

duration = refTrajObj.timeVector(end);
dt = refTrajObj.timeVector(2) - refTrajObj.timeVector(1);
len = length(refTrajObj.timeVector) - 1;
levelling_update_step = 1;

if strcmp(mdl.robotName, 'APAS_var0')
    horizontal_trans_idx = 16;
    vertical_trans_idx = 17;
    steering_jnt_idx = 9;
elseif strcmp(mdl.robotName, 'APASR_var0')
    horizontal_trans_idx = 28;
    vertical_trans_idx = 29;
    steering_jnt_idx = 9;
else
    horizontal_platform_idx = mdl.getLinkIndex('horizontal_moving_platform');
    vertical_platform_idx = mdl.getLinkIndex('vertical_moving_platform');
    steering_shaft_idx = mdl.getLinkIndex('steering_shaft');
    horizontal_trans_idx = mdl.dof_interval_links(2, horizontal_platform_idx);
    vertical_trans_idx = mdl.dof_interval_links(2, vertical_platform_idx);
    steering_jnt_idx = mdl.dof_interval_links(2, steering_shaft_idx);
end

% des_pose = pose_s;
% des_vel = vel0;
q0 = zeros(mdl.numDofs, 1);

if strcmp(mdl.robotName, 'APASR_var0')
    q0(10) = -0.15;
    q0(11) = 0.258;
    q0(12) = 0.363;

    q0(13) = -0.15;
    q0(14) = -0.258;
    q0(15) = -0.363;

    q0(16) = 0.15;
    q0(17) = -0.258;
    q0(18) = 0.363;

    q0(19) = 0.15;
    q0(20) = 0.258;
    q0(21) = -0.363;
end

qdot0 = zeros(mdl.numDofs, 1);
taskSpaceDescription_init = APASVar0TaskSpaceUpdate(mdl, q0, qdot0);
des_crane_cable_lengths = taskSpaceDescription_init.crane_cable_lengths;
des_module_cable_lengths = taskSpaceDescription_init.module_cable_lengths;
des_hoisting_cable_lengths = taskSpaceDescription_init.hoisting_cable_lengths;
module_attitude_euler_xyz_init = taskSpaceDescription_init.module_attitude_euler_xyz;
steering_jnt = q0(steering_jnt_idx);
horizontal_trans = q0(horizontal_trans_idx);
vertical_trans = q0(vertical_trans_idx);

% get the balancing starting pose
% % var1: 1.5*0.1 steering shaft cable attachment spread
% pose = [-2.24461470398532e-15;1.39628853505927e-15;-0.0392398909736771;-5.42808676335842e-15;-6.17808075264212e-16;-1.24202400305380e-15;-1.50751246324818e-15;-1.68590346568414e-15;8.64893641266428e-16;-1.13729504682101e-14;4.06528548148361e-14;-0.00487401460882458;7.08990267738826e-15;2.04100768468789e-15;3.85588140799655e-16;1.51213528114601e-16;-2.33214096314807e-16;-8.23401472590957e-16;-1.21121295623354e-15;-0.00980997185671761;-1.58406272783177e-16;-1.20092202239508e-16;-6.96314484345370e-18];
% vel = qdot0;
% var0: 1.5*0.05 steering shaft cable attachment spread
pose = [2.90070181551419e-09; 7.88351816575030e-15; -0.000981000000214328; -7.23649395966789e-16; -2.28621051895129e-13; -5.01016567338234e-15; -1.14510731277914e-14; -1.34203677470155e-10; 6.99983466068133e-15; 3.61162606832704e-10; 3.86095810082659e-14; -0.00328771208018424; 7.26212300299872e-15; -6.99896595558849e-11; -2.01633973348163e-15; -1.84855803083218e-11; 1.15623265758592e-15; -4.71442410305799e-10; 3.04222763174032e-14; -0.000981000004225588; -7.26158797556293e-18; -7.60907287398512e-14; 1.09494114181880e-17];
vel = qdot0;

%% stabilize the system

if ~bypass_stabilization_step

    stabilization_time = 50;
    stabilization_dt = 0.001;
    stabilization_len = round(stabilization_time / stabilization_dt);
    crane_cable_forces = zeros(2, 1);
    hoisting_cable_forces = zeros(4, 1);
    module_cable_forces = zeros(4, 1);
    steering_motor_torques = 0.0;
    linear_motor_forces = zeros(2, 1);
    thruster_forces = zeros(8, 1);

    for i = 1:stabilization_len

        i / stabilization_len
        taskSpaceDescription = APASVar0TaskSpaceUpdate(mdl, pose, vel);
        module_attitude_euler_xyz = taskSpaceDescription.module_attitude_euler_xyz;
        module_twist = taskSpaceDescription.module_twist;
        crane_cable_lengths = taskSpaceDescription.crane_cable_lengths;
        module_cable_lengths = taskSpaceDescription.module_cable_lengths;
        hoisting_cable_lengths = taskSpaceDescription.hoisting_cable_lengths;
        crane_cable_velocities = taskSpaceDescription.crane_cable_velocities;
        module_cable_velocities = taskSpaceDescription.module_cable_velocities;
        hoisting_cable_velocities = taskSpaceDescription.hoisting_cable_velocities;
        thruster_velocities = taskSpaceDescription.hoisting_frame_vel_along_thruster_vec;

        % calculate crane cable forces
        for j = 1:2
            crane_cable_forces(j) = crane_cable_ff(j) + crane_cable_cpid(j).execute(crane_cable_lengths(j), ...
                crane_cable_velocities(j), ...
                des_crane_cable_lengths(j), ...
                0.0);
        end

        % calculate module cable forces
        for j = 1:4
            module_cable_forces(j) = module_cable_ff(j) + module_cable_cpid(j).execute(module_cable_lengths(j), ...
                module_cable_velocities(j), ...
                des_module_cable_lengths(j), ...
                0.0);
        end

        % calculate hoisting cable forces
        for j = 1:4
            hoisting_cable_forces(j) = hoisting_cable_ff(j) + hoisting_cable_cpid(j).execute(hoisting_cable_lengths(j), ...
                hoisting_cable_velocities(j), ...
                des_hoisting_cable_lengths(j), ...
                0.0);
        end

        % calculate steering motor torque
        %     % pure steering joint
        %     steering_motor_torques = steering_motor_cpid.execute(pose(steering_jnt_idx),...
        %                                                                 vel(steering_jnt_idx),...
        %                                                                 steering_jnt,...
        %                                                                 0.0);
        % pure module steering
        steering_motor_torques = steering_motor_cpid.execute(module_attitude_euler_xyz(3) - module_attitude_euler_xyz_init(3), ...
            module_twist(3), ...
            0.0, ...
            0.0);

        % calculate linear motor forces
        des_XYTable_trans = [horizontal_trans; vertical_trans];

        for j = 1:2
            linear_motor_forces(j) = linear_motor_cpid(j).execute(pose(horizontal_trans_idx - 1 + j), ...
                vel(horizontal_trans_idx - 1 + j), ...
                des_XYTable_trans(j), ...
                0.0);
        end

        % calculate thruster forces
        for j = 1:8

            if abs(thruster_velocities(j)) < thruster_vel_threshold
                thruster_forces(j) = 0.0;
            else
                thruster_forces(j) = min([thruster_force_max, max([thruster_force_min, thruster_gain * thruster_velocities(j)])]);
            end

        end

        u = [crane_cable_forces;
             hoisting_cable_forces;
             module_cable_forces;
             steering_motor_torques;
             linear_motor_forces;
             thruster_forces];

        if sum(isnan(vel))
            wtf = true
        end

        damping_wrench = -0.0 * vel;

        if strcmp(mdl.robotName, 'APASR_var0')
            damping_wrench(10:21) = -0.001 * vel(10:21);
        end

        [pose, vel] = mdl.systemPropagation(pose, vel, u, damping_wrench, dt, 'ode4');

        if mod(i, 200) == 0
            % ModelVisualization.PlotFrame(mdl, plot_axis, view_angle);
        end

    end

end

q_rec = pose;
q_dot_rec = vel;

taskSpaceDescription = APASVar0TaskSpaceUpdate(mdl, pose, vel);

module_position_init = taskSpaceDescription.module_position;
module_attitude_euler_xyz_init = taskSpaceDescription.module_attitude_euler_xyz;

%% perform simulation

t = 0;
crane_cable_forces = zeros(len, 2);
module_cable_forces = zeros(len, 4);
hoisting_cable_forces = zeros(len, 4);
steering_motor_torques = zeros(len, 1);
linear_motor_forces = zeros(len, 2);
thruster_forces = zeros(len, 8);

crane_cable_errors = zeros(len, 2);
module_cable_errors = zeros(len, 4);
hoisting_cable_errors = zeros(len, 4);
steering_motor_errors = zeros(len, 1);
steering_motor_errors_dot = zeros(len, 1);
steering_motor_pos_des = zeros(len, 1);
steering_motor_pos_meas = zeros(len, 1);
steering_motor_vel_des = zeros(len, 1);
steering_motor_vel_meas = zeros(len, 1);
linear_motor_errors = zeros(len, 2);
thruster_velocity_errors = zeros(len, 8);

if strcmp(mdl.robotName, 'APASR_var0')
    cable1_hook_joint_pos = zeros(len, 3);
    cable2_hook_joint_pos = zeros(len, 3);
    cable3_hook_joint_pos = zeros(len, 3);
    cable4_hook_joint_pos = zeros(len, 3);
    cable1_hook_joint_vel = zeros(len, 3);
    cable2_hook_joint_vel = zeros(len, 3);
    cable3_hook_joint_vel = zeros(len, 3);
    cable4_hook_joint_vel = zeros(len, 3);
end

crane_cable_vel = zeros(len, 2);
module_cable_vel = zeros(len, 4);
hoisting_cable_vel = zeros(len, 4);

module_level = zeros(len, 2); % alpha beta from euler xyz representation of module attitude
module_steering_angle = zeros(len, 1); % gamma from euler xyz representation of module attitude
module_translation = zeros(len, 3);

module_level_des = zeros(len, 2);
module_steering_angle_des = zeros(len, 1);
module_translation_des = zeros(len, 3);

pose_profile = cell(len, 1);
time_profile = zeros(len, 1);

prev_steering_motor_torque = 0.0;

ModelVisualization.PlotFrame(mdl, plot_axis, view_angle);

des_pose_prev = refTrajObj.q{1};

for i = 1:len
    % update desired pose
    des_module_pose = refTrajObj.q{i + 1};
    des_module_vel = refTrajObj.q_dot{i + 1};

    module_translation_des(i, :) = des_module_pose(1:3)';
    module_steering_angle_des(i) = des_module_pose(4);
    module_level_des(i, :) = zeros(1, 2);

    des_steering_jnt_vel = des_module_vel(4);

    i / len
    pose_profile{i} = pose;
    time_profile(i) = i * dt;
    taskSpaceDescription = APASVar0TaskSpaceUpdate(mdl, pose, vel);
    crane_cable_lengths = taskSpaceDescription.crane_cable_lengths;
    module_cable_lengths = taskSpaceDescription.module_cable_lengths;
    hoisting_cable_lengths = taskSpaceDescription.hoisting_cable_lengths;
    crane_cable_velocities = taskSpaceDescription.crane_cable_velocities;
    module_cable_velocities = taskSpaceDescription.module_cable_velocities;
    hoisting_cable_velocities = taskSpaceDescription.hoisting_cable_velocities;
    thruster_velocities = taskSpaceDescription.hoisting_frame_vel_along_thruster_vec;

    module_twist = taskSpaceDescription.module_twist;

    module_position = taskSpaceDescription.module_position;
    module_attitude_euler_xyz = taskSpaceDescription.module_attitude_euler_xyz;

    module_translation(i, :) = module_position - module_position_init;
    module_steering_angle(i) = module_attitude_euler_xyz(3) - module_attitude_euler_xyz_init(3);
    module_level(i, :) = module_attitude_euler_xyz(1:2)';

    [steering_jnt, des_hoisting_cable_lengths, horizontal_trans, vertical_trans] = APASVar0StepModulePositionAndSteeringAngle(mdl, q0, des_module_pose);

    % calculate crane cable forces
    for j = 1:2
        crane_cable_errors(i, j) = des_crane_cable_lengths(j) - crane_cable_lengths(j);
        crane_cable_forces(i, j) = crane_cable_ff(j) + crane_cable_cpid(j).execute(crane_cable_lengths(j), ...
            crane_cable_velocities(j), ...
            des_crane_cable_lengths(j), ...
            0.0);
        crane_cable_vel(i, j) = crane_cable_velocities(j);
    end

    % calculate module cable forces
    for j = 1:4
        module_cable_errors(i, j) = des_module_cable_lengths(j) - module_cable_lengths(j);
        module_cable_forces(i, j) = module_cable_ff(j) + module_cable_cpid(j).execute(module_cable_lengths(j), ...
            module_cable_velocities(j), ...
            des_module_cable_lengths(j), ...
            0.0);
        module_cable_vel(i, j) = module_cable_velocities(j);
    end

    % calculate hoisting cable forces
    for j = 1:4
        hoisting_cable_errors(i, j) = des_hoisting_cable_lengths(j) - hoisting_cable_lengths(j);
        hoisting_cable_forces(i, j) = hoisting_cable_ff(j) + hoisting_cable_cpid(j).execute(hoisting_cable_lengths(j), ...
            hoisting_cable_velocities(j), ...
            des_hoisting_cable_lengths(j), ...
            0.0);
        hoisting_cable_vel(i, j) = hoisting_cable_velocities(j);
    end

    % calculate steering motor torque
    %     % pure steering joint
    %     steering_motor_pos_des(i,1) = steering_jnt;
    %     steering_motor_pos_meas(i,1) = pose(steering_jnt_idx);
    %     steering_motor_vel_des(i,1) = des_steering_jnt_vel;
    %     steering_motor_vel_meas(i,1) = vel(steering_jnt_idx);
    %     steering_motor_errors(i,1) = steering_jnt - pose(steering_jnt_idx);
    %     steering_motor_errors_dot(i,1) = des_steering_jnt_vel - vel(steering_jnt_idx);
    %     steering_motor_torques(i,1) = steering_motor_cpid.execute(pose(steering_jnt_idx),...
    %                                                                 vel(steering_jnt_idx),...
    %                                                                 steering_jnt,...
    %                                                                 des_steering_jnt_vel);
    %     % pure module steering
    %     steering_motor_pos_des(i,1) = des_module_pose(4);
    %     steering_motor_pos_meas(i,1) = module_steering_angle(i);
    %     steering_motor_vel_des(i,1) = des_steering_jnt_vel;
    %     steering_motor_vel_meas(i,1) = module_twist(3);
    %     steering_motor_errors(i,1) = des_module_pose(4) - module_steering_angle(i);
    %     steering_motor_errors_dot(i,1) = des_steering_jnt_vel - module_twist(3);
    %     steering_motor_torques(i,1) = steering_motor_cpid.execute(module_steering_angle(i),...
    %                                                                 module_twist(3),...
    %                                                                 des_module_pose(4),...
    %                                                                 des_steering_jnt_vel);
    % combination of steering joint and module steering
    steering_channel_vel_weighted_sum = (1 - joint_feedback_weight) * module_twist(3) + joint_feedback_weight * vel(steering_jnt_idx);
    steering_motor_pos_des(i, 1) = des_module_pose(4);
    steering_motor_pos_meas(i, 1) = module_steering_angle(i);
    steering_motor_vel_des(i, 1) = des_steering_jnt_vel;
    steering_motor_vel_meas(i, 1) = steering_channel_vel_weighted_sum;
    steering_motor_errors(i, 1) = des_module_pose(4) - module_steering_angle(i);
    steering_motor_errors_dot(i, 1) = des_steering_jnt_vel - steering_channel_vel_weighted_sum;
    steering_motor_torques(i, 1) = (1 - steering_motor_torque_lpf) * prev_steering_motor_torque + steering_motor_torque_lpf * steering_motor_cpid.execute(module_steering_angle(i), ...
        steering_channel_vel_weighted_sum, ...
        des_module_pose(4), ...
        des_steering_jnt_vel);
    prev_steering_motor_torque = steering_motor_torques(i, 1);

    % calculate linear motor forces
    des_XYTable_trans = [horizontal_trans; vertical_trans];

    for j = 1:2
        linear_motor_errors(i, j) = des_XYTable_trans(j) - pose(horizontal_trans_idx - 1 + j);
        linear_motor_forces(i, j) = linear_motor_cpid(j).execute(pose(horizontal_trans_idx - 1 + j), ...
            vel(horizontal_trans_idx - 1 + j), ...
            des_XYTable_trans(j), ...
            0.0);
    end

    % calculate thruster forces
    for j = 1:8
        thruster_velocity_errors(i, j) = thruster_velocities(j);

        if abs(thruster_velocities(j)) < thruster_vel_threshold
            thruster_forces(i, j) = 0.0;
        else
            thruster_forces(i, j) = min([thruster_force_max, max([thruster_force_min, thruster_gain * thruster_velocities(j)])]);
        end

    end

    if strcmp(mdl.robotName, 'APASR_var0')
        cable1_hook_joint_pos(i, :) = pose(10:12)';
        cable2_hook_joint_pos(i, :) = pose(13:15)';
        cable3_hook_joint_pos(i, :) = pose(16:18)';
        cable4_hook_joint_pos(i, :) = pose(19:21)';
        cable1_hook_joint_vel(i, :) = vel(10:12)';
        cable2_hook_joint_vel(i, :) = vel(13:15)';
        cable3_hook_joint_vel(i, :) = vel(16:18)';
        cable4_hook_joint_vel(i, :) = vel(19:21)';
    end

    u = [crane_cable_forces(i, :), ...
             hoisting_cable_forces(i, :), ...
             module_cable_forces(i, :), ...
             steering_motor_torques(i, 1), ...
             linear_motor_forces(i, :), ...
             thruster_forces(i, :)]';

    if sum(isnan(vel))
        wtf = true
    end

    damping_wrench = -0.0 * vel;

    if strcmp(mdl.robotName, 'APASR_var0')
        damping_wrench(10:21) = -0.00 * vel(10:21);
    end

    [pose, vel] = mdl.systemPropagation(pose, vel, u, damping_wrench, dt, 'ode4');

    if mod(i, 200) == 0
        % ModelVisualization.PlotFrame(mdl, plot_axis, view_angle);
    end

end

%%

time_vec = refTrajObj.timeVector(2:end);

ModelVisualization.PlotFrame(mdl, plot_axis, view_angle);

if strcmp(mdl.robotName, 'APASR_var0')
    figure;
    plot(cable1_hook_joint_pos);
    title('hook joint pos of hoisting cable 1');
    figure;
    plot(cable2_hook_joint_pos);
    title('hook joint pos of hoisting cable 2');
    figure;
    plot(cable3_hook_joint_pos);
    title('hook joint pos of hoisting cable 3');
    figure;
    plot(cable4_hook_joint_pos);
    title('hook joint pos of hoisting cable 4');

    figure;
    plot(cable1_hook_joint_vel);
    title('hook joint vel of hoisting cable 1');
    figure;
    plot(cable2_hook_joint_vel);
    title('hook joint vel of hoisting cable 2');
    figure;
    plot(cable3_hook_joint_vel);
    title('hook joint vel of hoisting cable 3');
    figure;
    plot(cable4_hook_joint_vel);
    title('hook joint vel of hoisting cable 4');
end

%%
font_name = 'Times';
general_font_size = 14;
axis_tick_font_size = 10;

LineStyleLib = {'-', '--', ':', '-.'};
bg = {'k', 'w'};
colors = distinguishable_colors(8);

figure;
hp = plot(time_vec, crane_cable_errors);
grid on;
legend('C1', 'C2');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (m)', 'FontSize', general_font_size, 'FontName', font_name);
title('Crane Cable Errors');
% ylim([-1,1]);

% figure;
% plot(-crane_cable_vel);
% title('Crane Cable Velocity Errors');
% ylim([-1,1]);

figure;
hp = plot(time_vec, crane_cable_forces);
grid on;
legend('C1', 'C2');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Force (N)', 'FontSize', general_font_size, 'FontName', font_name);
title('Crane Cable Forces');

figure;
hp = plot(time_vec, module_cable_errors);
grid on;
legend('M1', 'M2', 'M3', 'M4');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (m)', 'FontSize', general_font_size, 'FontName', font_name);
title('Module Cable Errors');
% ylim([-1,1]);

% figure;
% plot(-module_cable_vel);
% title('module cable vel errors');
% ylim([-1,1]);

figure;
hp = plot(time_vec, module_cable_forces);
grid on;
legend('M1', 'M2', 'M3', 'M4');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Force (N)', 'FontSize', general_font_size, 'FontName', font_name);
title('Module Cable Forces');

figure;
hp = plot(time_vec, hoisting_cable_errors);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$l_1$', '$l_2$', '$l_3$', '$l_4$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
% legend('H1', 'H2', 'H3', 'H4');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (m)', 'FontSize', general_font_size, 'FontName', font_name);
title('Hoisting Cable Errors');
% ylim([-1,1]);

% figure;
% plot(-hoisting_cable_vel);
% title('hoisting cable vel errors');
% ylim([-1,1]);

figure;
hp = plot(time_vec, hoisting_cable_forces);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$l_1$', '$l_2$', '$l_3$', '$l_4$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
% legend('H1', 'H2', 'H3', 'H4');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Force (N)', 'FontSize', general_font_size, 'FontName', font_name);
title('Hoisting Cable Forces');

%%

figure;
hp = plot(time_vec, linear_motor_errors);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'LinearX', 'LinearY'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
% legend('LinearX', 'LinearY');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (m)', 'FontSize', general_font_size, 'FontName', font_name);
title('Linear Motor Errors');
% ylim([-1,1]);

figure;
hp = plot(time_vec, linear_motor_forces);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'LinearX', 'LinearY'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
% legend('LinearX', 'LinearY');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Force (N)', 'FontSize', general_font_size, 'FontName', font_name);
title('Linear Motor Forces');

figure;
hp = plot(time_vec, rad2deg(steering_motor_errors));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    % hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    % hl.FontSize = general_font_size;
    % set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (deg)', 'FontSize', general_font_size, 'FontName', font_name);
title('Steering Motor Error');
% ylim([-1,1]);

figure;
hp = plot(time_vec, rad2deg(steering_motor_pos_des));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    % hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    % hl.FontSize = general_font_size;
    % set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Position (deg)', 'FontSize', general_font_size, 'FontName', font_name);
title('Steering Motor Desired Position');
% ylim([-1,1]);

figure;
hp = plot(time_vec, rad2deg(steering_motor_pos_meas));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    % hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    % hl.FontSize = general_font_size;
    % set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Position (deg)', 'FontSize', general_font_size, 'FontName', font_name);
title('Steering Motor Measured Position');
% ylim([-1,1]);

figure;
hp = plot(time_vec, rad2deg(steering_motor_vel_des));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    % hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    % hl.FontSize = general_font_size;
    % set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Velocity (deg/sec)', 'FontSize', general_font_size, 'FontName', font_name);
title('Steering Motor Desired Velocity');
% ylim([-1,1]);

figure;
hp = plot(time_vec, rad2deg(steering_motor_vel_meas));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    % hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    % hl.FontSize = general_font_size;
    % set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Velocity (deg/sec)', 'FontSize', general_font_size, 'FontName', font_name);
title('Steering Motor Measured Velocity');
% ylim([-1,1]);

figure;
hp = plot(time_vec, rad2deg(steering_motor_errors_dot));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    % hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    % hl.FontSize = general_font_size;
    % set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error Derivative (deg/sec)', 'FontSize', general_font_size, 'FontName', font_name);
title('Steering Motor Error Derivative');
% ylim([-1,1]);

figure;
hp = plot(time_vec, steering_motor_torques);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    % hp(i).DisplayName = name_list{i};
    % hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    % hl.FontSize = general_font_size;
    % set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Torque (Nm)', 'FontSize', general_font_size, 'FontName', font_name);
title('Steering Motor Torue');

figure;
hp = plot(time_vec, thruster_velocity_errors);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Velocity (m/s)', 'FontSize', general_font_size, 'FontName', font_name);
% legend('T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8');
title('Thruster Axial Velocities');

figure;
hp = plot(time_vec, thruster_forces);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$t_1$', '$t_2$', '$t_3$', '$t_4$', '$t_5$', '$t_6$', '$t_7$', '$t_8$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'eastoutside');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Thrust (N)', 'FontSize', general_font_size, 'FontName', font_name);
% legend('T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7', 'T8');
title('Thruster Forces');

%%

figure;
hp = plot(time_vec, module_translation);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$X$', '$Y$', '$Z$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

hold on;
hp = plot(time_vec, module_translation_des, ':');
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$X_d$', '$Y_d$', '$Z_d$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{2};
    hp(i).LineWidth = 2.5;
    hp(i).Color = 0.7 * colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
% legend('$X$', '$Y$', '$Z$', '$X_d$', '$Y_d$', '$Z_d$', 'Location', 'best');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Position (m)', 'FontSize', general_font_size, 'FontName', font_name);
title('Module Translation Tracking');

figure;
hp = plot(time_vec, rad2deg(module_steering_angle));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$\gamma$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

hold on;
hp = plot(time_vec, rad2deg(module_steering_angle_des), ':');
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$\gamma_d$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{2};
    hp(i).LineWidth = 2.5;
    hp(i).Color = 0.7 * colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
% legend('$\gamma$', '$\gamma_d$', 'Location', 'best');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (deg)', 'FontSize', general_font_size, 'FontName', font_name);
title('Module Steering Angle Tracking');

figure;
hp = plot(time_vec, module_translation_des - module_translation);
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$X$', '$Y$', '$Z$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
% legend('$X$', '$Y$', '$Z$', 'Location', 'best');
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (m)', 'FontSize', general_font_size, 'FontName', font_name);
title('Module Translation Error');

figure;
hp = plot(time_vec, rad2deg(module_steering_angle_des - module_steering_angle));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
name_list = {'$\tilde{\gamma}$'};

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    hp(i).DisplayName = name_list{i};
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (deg)', 'FontSize', general_font_size, 'FontName', font_name);
title('Module Steering Angle Error');

figure;
hp = plot(time_vec, rad2deg(module_level));
ax = gca;
ax.YAxis.FontSize = axis_tick_font_size;
ax.XAxis.FontSize = axis_tick_font_size;
ax.FontName = font_name;
legend('$\alpha$', '$\beta$');

for i = 1:length(hp)
    hp(i).LineStyle = LineStyleLib{1};
    hp(i).LineWidth = 1.5;
    hp(i).Color = colors(i, :);
    % hp(i).DisplayName = strcat('$f_', num2str(i),'$');
    hl = legend('show', 'FontName', font_name, 'Location', 'best');
    hl.FontSize = general_font_size;
    set(hl, 'Interpreter', 'latex');
end

grid on;
xlabel('Time (sec)', 'FontSize', general_font_size, 'FontName', font_name);
ylabel('Error (deg)', 'FontSize', general_font_size, 'FontName', font_name);
title('Module Levelling Error');

%% movie

% view_angle = [0 -90]; % top view
% view_angle = [0 0]; % front view
% view_angle = [90 0]; % side view
view_angle = [30 30]; % natural view
content_string = 'natural_view';
script_path = fileparts(mfilename('fullpath'));
folder_name = strcat('\');
filename = [content_string, '.avi'];
traj_visualization_set = [];
q_ref_series = [];
time = 1 * time_profile(end);
ModelVisualization.PlotMovie(mdl, pose_profile, time_profile, plot_axis, view_angle, folder_name, filename, traj_visualization_set, q_ref_series, time);

view_angle = [0 -90]; % top view
% view_angle = [0 0]; % front view
% view_angle = [90 0]; % side view
% view_angle = [30 30]; % natural view
content_string = 'top_view';
script_path = fileparts(mfilename('fullpath'));
folder_name = strcat('\');
filename = [content_string, '.avi'];
traj_visualization_set = [];
q_ref_series = [];
time = 1 * time_profile(end);
ModelVisualization.PlotMovie(mdl, pose_profile, time_profile, plot_axis, view_angle, folder_name, filename, traj_visualization_set, q_ref_series, time);

% view_angle = [0 -90]; % top view
view_angle = [0 0]; % front view
% view_angle = [90 0]; % side view
% view_angle = [30 30]; % natural view
content_string = 'front_view';
script_path = fileparts(mfilename('fullpath'));
folder_name = strcat('\');
filename = [content_string, '.avi'];
traj_visualization_set = [];
q_ref_series = [];
time = 1 * time_profile(end);
ModelVisualization.PlotMovie(mdl, pose_profile, time_profile, plot_axis, view_angle, folder_name, filename, traj_visualization_set, q_ref_series, time);

% view_angle = [0 -90]; % top view
% view_angle = [0 0]; % front view
view_angle = [90 0]; % side view
% view_angle = [30 30]; % natural view
content_string = 'side_view';
script_path = fileparts(mfilename('fullpath'));
folder_name = strcat('\');
filename = [content_string, '.avi'];
traj_visualization_set = [];
q_ref_series = [];
time = 1 * time_profile(end);
ModelVisualization.PlotMovie(mdl, pose_profile, time_profile, plot_axis, view_angle, folder_name, filename, traj_visualization_set, q_ref_series, time);
