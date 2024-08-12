function [res_valid, trans_err, attd_err_euler, pose] = APASStabilizeAtSetpoint( ...
        mdl_ctrl, ...
        mdl_sim, ...
        apas_task_space_update_func, ...
        apas_command_gen_func, ...
        q0_ctrl, ...
        q0_sim, ...
        des_module_pose, ...
        steering_joint_idx_set, ...
        linear_joint_idx_set, ...
        crane_cable_controllers, ...
        module_cable_controllers, ...
        hoisting_cable_controllers, ...
        steering_motor_controllers, ...
        linear_motor_controllers, ...
        thruster_gains, ...
        delta_t, ...
        t_max, ...
        err_threshold_trans, ...
        err_threshold_rot, ...
        t_termination_check_horizon)

    if (nargin < 18)
        err_threshold_trans = 5e-4; %  0.5 mm
        err_threshold_rot = 1e-3; % ~0.05 deg
        t_termination_check_horizon = 1.0;
    end

    err_debouncer = 200;
    err_debouncer_counter = 0;

    res_valid = false;

    numCraneCables = length(crane_cable_controllers);
    numHoistingCables = length(hoisting_cable_controllers);
    numModuleCables = length(module_cable_controllers);
    numSteeringMotors = length(steering_motor_controllers);
    numLinearMotors = length(linear_motor_controllers);
    numThrusters = length(thruster_gains);

    numDofs = length(q0_sim);
    pose = q0_sim;
    vel = zeros(numDofs, 1);

    % define thruster damper
    thruster_vel_threshold = 0.0;
    thruster_gain = 200.0;
    thruster_force_max = 50.0;
    thruster_force_min = -0.0;

    duration = t_max;
    dt = delta_t;
    len = round(duration / dt);
    termination_check_len = round(t_termination_check_horizon / dt);

    trans_err_termination_check = zeros(termination_check_len, 1);
    attd_err_termination_check = zeros(termination_check_len, 1);
    termination_check_ptr = 0;

    crane_cable_forces = zeros(numCraneCables, 1);
    hoisting_cable_forces = zeros(numHoistingCables, 1);
    module_cable_forces = zeros(numModuleCables, 1);
    steering_motor_torques = zeros(numSteeringMotors, 1);
    linear_motor_forces = zeros(numLinearMotors, 1);
    thruster_forces = zeros(numThrusters, 1);

    trans_err = zeros(3, 1);
    attd_err_euler = zeros(3, 1);
    trans_err_prev = zeros(3, 1);
    attd_err_euler_prev = zeros(3, 1);

    des_module_vel = zeros(size(des_module_pose));
    taskSpaceDescriptionInit = apas_task_space_update_func(mdl_ctrl, q0_ctrl, vel);
    crane_cable_len_des = taskSpaceDescriptionInit.crane_cable_lengths;
    module_cable_len_des = taskSpaceDescriptionInit.module_cable_lengths;
    [~, hoisting_cable_len_des, horizontal_trans, vertical_trans] = apas_command_gen_func(mdl_ctrl, q0_ctrl, des_module_pose);
    linear_motor_des = [horizontal_trans, vertical_trans]; % hardcoded stuff, don't like it

    taskSpaceDescriptionInit = apas_task_space_update_func(mdl_sim, q0_sim, vel);
    module_position_init = taskSpaceDescriptionInit.module_position;
    % modify q0_sim to suit the desired module pose
    q0_sim(10:12) = q0_sim(10:12) + des_module_pose(1:3) - [linear_motor_des'; 0.0];
    q0_sim(16:17) = q0_sim(16:17) + linear_motor_des';

    pose = q0_sim;
    vel = zeros(numDofs, 1);

    for i = 1:len
        % update termination check container pointer
        termination_check_ptr = mod(i, termination_check_len) + 1;

        taskSpaceDescription = apas_task_space_update_func(mdl_sim, pose, vel);
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

        trans_err = module_position - module_position_init - des_module_pose(1:3);
        attd_err_euler = module_attitude_euler_xyz - [0.0; 0.0; des_module_pose(4)];

        %  update termination check container and grab statistics for termination check
        trans_err_magnitude = norm(trans_err);
        attd_err_magnitude = norm(attd_err_euler);
        trans_err_termination_check(termination_check_ptr) = trans_err_magnitude;
        attd_err_termination_check(termination_check_ptr) = attd_err_magnitude;
        trans_err_mean = mean(trans_err_termination_check);
        attd_err_mean = mean(attd_err_termination_check);
        trans_err_max = max(trans_err_termination_check);
        attd_err_max = max(attd_err_termination_check);

        if norm(trans_err_mean) > 0.2 || norm(attd_err_mean) > 0.5
            res_valid = -1;
            trans_err_mean
            attd_err_mean
            return;
        end

        % trans_err_magnitude_change = norm(trans_err_prev - trans_err)
        % attd_err_magnitude_change = norm(attd_err_euler_prev - attd_err_euler)

        if ((trans_err_max - trans_err_mean) < err_threshold_trans && ...
                norm(attd_err_max - attd_err_mean) < err_threshold_rot)
            err_debouncer_counter = err_debouncer_counter + 1;
        else
            err_debouncer_counter = 0;
        end

        if err_debouncer_counter > err_debouncer
            final_t = dt * i
            trans_err_magnitude = norm(trans_err)
            attd_err_magnitude = norm(attd_err_euler)
            res_valid = true;
            trans_err_mean
            attd_err_mean
            return;
        end

        damping_wrench = -10.0 * vel;

        trans_err_prev = trans_err;
        attd_err_euler_prev = attd_err_euler;

        % calculate crane cable forces
        for j = 1:numCraneCables
            crane_cable_forces(j) = crane_cable_controllers(j).execute( ...
                crane_cable_lengths(j), ...
                crane_cable_velocities(j), ...
                crane_cable_len_des(j), ...
                0.0);
        end

        % calculate module cable forces
        for j = 1:numModuleCables
            module_cable_forces(j) = module_cable_controllers(j).execute( ...
                module_cable_lengths(j), ...
                module_cable_velocities(j), ...
                module_cable_len_des(j), ...
                0.0);
        end

        % calculate hoisting cable forces
        for j = 1:numHoistingCables
            hoisting_cable_forces(j) = hoisting_cable_controllers(j).execute( ...
                hoisting_cable_lengths(j), ...
                hoisting_cable_velocities(j), ...
                hoisting_cable_len_des(j), ...
                0.0);
        end

        % calculate steering motor torque
        for j = 1:numSteeringMotors
            % % pure steering joint
            % steering_motor_torques(j) = steering_motor_controllers(j).execute( ...
            %     pose(steering_joint_idx_set(j)), ...
            %     vel(steering_joint_idx_set(j)), ...
            %     des_module_pose(4), ...
            %     des_module_vel(4));
            % module steering angle
            steering_motor_torques(j) = steering_motor_controllers(j).execute( ...
                module_attitude_euler_xyz(3), ...
                module_twist(3), ...
                des_module_pose(4), ...
                des_module_vel(4));
        end

        % calculate linear motor forces
        for j = 1:numLinearMotors
            linear_motor_forces(j) = linear_motor_controllers(j).execute( ...
                pose(linear_joint_idx_set(j)), ...
                vel(linear_joint_idx_set(j)), ...
                linear_motor_des(j), ...
                0.0);
        end

        % calculate thruster forces
        for j = 1:numThrusters

            if abs(thruster_velocities(j)) < thruster_vel_threshold
                thruster_forces(j) = 0.0;
            else
                thruster_forces(j) = min([thruster_force_max, max([thruster_force_min, thruster_gains(j) * thruster_velocities(j)])]);
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

        [pose, vel] = mdl_sim.systemPropagation(pose, vel, u, damping_wrench, dt, 'ode4');

        if i == 1
            % plot_axis = [-0.2 10.2 -0.2 10.2 -0.1 60.1];
            % view_angle = [45 0];
            % ModelVisualization.PlotFrame(mdl_sim, plot_axis, view_angle);
        end

    end

    trans_err_mean
    attd_err_mean
end
