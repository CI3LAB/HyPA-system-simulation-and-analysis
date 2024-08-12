function taskSpaceDescription = APASVar2TaskSpaceUpdate(mdl, q, vel)
    %     taskSpaceDescription.pyramid_apex_height_in_hoisting_frame
    %     taskSpaceDescription.pyramid_apex_in_hoisting_frame
    %     taskSpaceDescription.g_vec_in_hoisting_frame
    %     taskSpaceDescription.quat_of_hoisting_frame
    %     taskSpaceDescription.hoisting_cable_lengths
    %     taskSpaceDescription.crane_cable_lengths
    %     taskSpaceDescription.module_cable_lengths
    %     taskSpaceDescription.hoisting_frame_vel_along_thruster_vec
    %     taskSpaceDescription.hoisting_cable_velocities
    %     taskSpaceDescription.crane_cable_velocities
    %     taskSpaceDescription.module_cable_velocities

    %     taskSpaceDescription.hoisting_frame_mass
    %     taskSpaceDescription.horizontal_frame_mass
    %     taskSpaceDescription.vertical_frame_mass
    %     taskSpaceDescription.module_mass
    %     taskSpaceDescription.hoisting_frame_angular_velocity

    %     taskSpaceDescription.module_position
    %     taskSpaceDescription.module_attitudd_quat
    %     taskSpaceDescription.module_attitudd_euler_xyz
    %     taskSpaceDescription.module_attitudd_euler_alpha
    %     taskSpaceDescription.module_attitudd_euler_beta
    %     taskSpaceDescription.module_attitudd_euler_gamma
    %     taskSpaceDescription.module_twist

    % update the model
    mdl.update(q, vel, zeros(size(vel)), zeros(size(vel)));

    % calculate cable lengths
    crane_cable_idx_set = [1, 2];
    module_cable_idx_set = [7, 8, 9, 10];
    hoisting_cable_idx_set = [3, 4, 5, 6];
    taskSpaceDescription.crane_cable_lengths = zeros(length(crane_cable_idx_set), 1);
    taskSpaceDescription.hoisting_cable_lengths = zeros(length(hoisting_cable_idx_set), 1);
    taskSpaceDescription.module_cable_lengths = zeros(length(module_cable_idx_set), 1);

    for i = 1:length(crane_cable_idx_set)
        cable_attachment_idx = crane_cable_idx_set(i);
        taskSpaceDescription.crane_cable_lengths(i) = mdl.cableLength(cable_attachment_idx);
    end

    for i = 1:length(module_cable_idx_set)
        cable_attachment_idx = module_cable_idx_set(i);
        taskSpaceDescription.module_cable_lengths(i) = mdl.cableLength(cable_attachment_idx);
    end

    for i = 1:length(hoisting_cable_idx_set)
        cable_attachment_idx = hoisting_cable_idx_set(i);
        taskSpaceDescription.hoisting_cable_lengths(i) = mdl.cableLength(cable_attachment_idx);
    end

    % calculate the hoisting cable pyramid apex
    if strcmp(mdl.robotName, 'APAS_var2')
        hoisting_frame_link_idx = 4;
        horizontal_frame_link_idx = 5;
        vertical_frame_link_idx = 6;
        module_link_idx = 7;
        steering_shaft_link_idx = 3;
    elseif strcmp(mdl.robotName, 'APASR_var2')
        hoisting_frame_link_idx = 17;
        horizontal_frame_link_idx = 18;
        vertical_frame_link_idx = 19;
        module_link_idx = 20;
        steering_shaft_link_idx = 4;
    else
        hoisting_frame_link_idx = mdl.getLinkIndex('hoisting_platform');
        horizontal_frame_link_idx = mdl.getLinkIndex('horizontal_moving_platform');
        vertical_frame_link_idx = mdl.getLinkIndex('vertical_moving_platform');
        module_link_idx = mdl.getLinkIndex('module');
        steering_shaft_link_idx = mdl.getLinkIndex('steering_shaft');
    end

    measuring_pt_local_coordinate = [0.0; 0.0; -0.1];
    apex_estimate = mdl.pointCoordinate(steering_shaft_link_idx, measuring_pt_local_coordinate, hoisting_frame_link_idx);

    taskSpaceDescription.pyramid_apex_in_hoisting_frame = apex_estimate;
    taskSpaceDescription.pyramid_apex_height_in_hoisting_frame = taskSpaceDescription.pyramid_apex_in_hoisting_frame(3);

    % calculate link quat for the hoisting frame
    taskSpaceDescription.quat_of_hoisting_frame = mdl.linkOrientationQuaternion(hoisting_frame_link_idx);

    % calculate gravity direction in hoisting frame
    R = QuaternionToRotationMatrix(taskSpaceDescription.quat_of_hoisting_frame);
    % axang = QuaternionToAxisAngle(taskSpaceDescription.quat_of_hoisting_frame);
    g_vec_in_ground = [0; 0; -1.0];
    taskSpaceDescription.g_vec_in_hoisting_frame = R * g_vec_in_ground;

    % calculate the hoisting frame velocity along each thruster direction
    % vector
    num_thrusters = length(mdl.propellers);
    taskSpaceDescription.hoisting_frame_vel_along_thruster_vec = zeros(num_thrusters, 1);

    for i = 1:num_thrusters
        vel_at_thruster_coordinate = mdl.propellerVelocity(i);
        thruster_airflow_direction = mdl.propeller_data_directions(:, mdl.propellers(i));
        taskSpaceDescription.hoisting_frame_vel_along_thruster_vec(i) = dot(vel_at_thruster_coordinate, thruster_airflow_direction);
    end

    l_dot = mdl.L * vel;
    taskSpaceDescription.crane_cable_velocities = l_dot(crane_cable_idx_set);
    taskSpaceDescription.hoisting_cable_velocities = l_dot(hoisting_cable_idx_set);
    taskSpaceDescription.module_cable_velocities = l_dot(module_cable_idx_set);

    taskSpaceDescription.hoisting_frame_mass = mdl.m(hoisting_frame_link_idx);
    taskSpaceDescription.horizontal_frame_mass = mdl.m(horizontal_frame_link_idx);
    taskSpaceDescription.vertical_frame_mass = mdl.m(vertical_frame_link_idx);
    taskSpaceDescription.module_mass = mdl.m(module_link_idx);
    taskSpaceDescription.hoisting_frame_angular_velocity = mdl.orientationJacobian(hoisting_frame_link_idx) * vel;

    module_point_of_interest = [0.0; 0.0; 0.0];
    module_quat = mdl.linkOrientationQuaternion(module_link_idx);
    [module_alpha, module_beta, module_gamma] = mdl.linkOrientationEulerXYZ(module_link_idx);
    taskSpaceDescription.module_position = mdl.pointCoordinate(module_link_idx, module_point_of_interest, 0);
    taskSpaceDescription.module_attitude_quat = module_quat;
    taskSpaceDescription.module_attitude_euler_xyz = [module_alpha; module_beta; module_gamma];
    taskSpaceDescription.module_attitude_euler_alpha = module_alpha;
    taskSpaceDescription.module_attitude_euler_beta = module_beta;
    taskSpaceDescription.module_attitude_euler_gamma = module_gamma;

    [moduleJacobian, ~] = mdl.linkJacobian(module_link_idx);
    taskSpaceDescription.module_twist = moduleJacobian * vel;
end
