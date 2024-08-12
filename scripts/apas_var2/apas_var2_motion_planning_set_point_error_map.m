clc; clear; close all;
%% build the model
def_apas_var2;
% def_apas_realistic;
mdl_ctrl = RigidBodySystem(model_def);

mdl_sim = RigidBodySystem(model_def);

%% create the trajectory
% traj_APAS_Module_rest;
% traj_APAS_Module_move_and_rest;
% traj_APAS_Module_steer_and_rest;
% traj_APAS_Module_move_slow_and_rest;
traj_APAS_Module_move_slow_and_rest_long;
refTrajObj = JointSpaceTrajectory(traj_def);

%% create controllers

bypass_stabilization_step = true;

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
thruster_gain = 5000.0;
% thruster_gain = 50.0;
% thruster_gain = 0000.0;
thruster_force_max = 200.0;
thruster_force_min = -0.0;

fmin = 0.0;

if strcmp(mdl_sim.robotName, 'APAS_var2')
    crane_cable_load = 10 * sum(mdl_sim.m(:)) / 2.0
    hoisting_cable_load = 10 * sum(mdl_sim.m(4:end)) / 4.0
    module_cable_load = 10 * sum(mdl_sim.m(end)) / 4.0
elseif strcmp(mdl_sim.robotName, 'APASR_var2')
    crane_cable_load = 10 * sum(mdl_sim.m(:)) / 2.0
    hoisting_cable_load = 10 * sum(mdl_sim.m(17:end)) / 4.0
    module_cable_load = 10 * sum(mdl_sim.m(end)) / 4.0
end

positionLoopPower = 1.0;
velocityLoopPower = 1.0;
crane_cable_u_max = crane_cable_load * load_factor_crane_cable;
crane_cable_ep_max = 0.01;
% crane_cable_ep_max = 0.001;
crane_cable_ev_max = 0.1;
crane_cable_kp = crane_cable_load / crane_cable_ep_max;
crane_cable_kd = 5e4;
crane_cable_kd = damping_ratio_crane_cable * sqrt(crane_cable_kp);
% crane_cable_u_max = 5e4;
% crane_cable_ep_max = 10;
% crane_cable_ev_max = 10;
% crane_cable_kp = 1e6;
% crane_cable_kd = 5e3;
crane_cable_pid(1) = PID(crane_cable_u_max, crane_cable_ep_max * len_error_scalar_crane_cable, crane_cable_ev_max * vel_error_scalar_crane_cable, crane_cable_kp, crane_cable_kd);
crane_cable_pid(2) = PID(crane_cable_u_max, crane_cable_ep_max * len_error_scalar_crane_cable, crane_cable_ev_max * vel_error_scalar_crane_cable, crane_cable_kp, crane_cable_kd);
crane_cable_pid(1).setUmin(fmin);
crane_cable_pid(2).setUmin(fmin);
crane_cable_pid(1).setGainCurve(positionLoopPower, velocityLoopPower);
crane_cable_pid(2).setGainCurve(positionLoopPower, velocityLoopPower);
crane_cable_pid(1).setCableLengthController();
crane_cable_pid(2).setCableLengthController();

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
hoisting_cable_pid(1) = PID(hoisting_cable_u_max, hoisting_cable_ep_max * len_error_scalar_hoisting_cable, hoisting_cable_ev_max * vel_error_scalar_hoisting_cable, hoisting_cable_kp, hoisting_cable_kd);
hoisting_cable_pid(2) = PID(hoisting_cable_u_max, hoisting_cable_ep_max * len_error_scalar_hoisting_cable, hoisting_cable_ev_max * vel_error_scalar_hoisting_cable, hoisting_cable_kp, hoisting_cable_kd);
hoisting_cable_pid(3) = PID(hoisting_cable_u_max, hoisting_cable_ep_max * len_error_scalar_hoisting_cable, hoisting_cable_ev_max * vel_error_scalar_hoisting_cable, hoisting_cable_kp, hoisting_cable_kd);
hoisting_cable_pid(4) = PID(hoisting_cable_u_max, hoisting_cable_ep_max * len_error_scalar_hoisting_cable, hoisting_cable_ev_max * vel_error_scalar_hoisting_cable, hoisting_cable_kp, hoisting_cable_kd);
hoisting_cable_pid(1).setUmin(fmin);
hoisting_cable_pid(2).setUmin(fmin);
hoisting_cable_pid(3).setUmin(fmin);
hoisting_cable_pid(4).setUmin(fmin);
hoisting_cable_pid(1).setGainCurve(positionLoopPower, velocityLoopPower);
hoisting_cable_pid(2).setGainCurve(positionLoopPower, velocityLoopPower);
hoisting_cable_pid(3).setGainCurve(positionLoopPower, velocityLoopPower);
hoisting_cable_pid(4).setGainCurve(positionLoopPower, velocityLoopPower);
hoisting_cable_pid(1).setCableLengthController();
hoisting_cable_pid(2).setCableLengthController();
hoisting_cable_pid(3).setCableLengthController();
hoisting_cable_pid(4).setCableLengthController();

positionLoopPower = 1.0;
velocityLoopPower = 1.0;
module_cable_u_max = module_cable_load * load_factor_module_cable;
module_cable_ep_max = 0.01;
% module_cable_ep_max = 0.001;
module_cable_ev_max = 0.1;
module_cable_kp = module_cable_load / module_cable_ep_max;
module_cable_kd = 1e4;
module_cable_kd = damping_ratio_module_cable * sqrt(module_cable_kp);
% module_cable_u_max = 5e4;
% module_cable_ep_max = 10;
% module_cable_ev_max = 10;
% module_cable_kp = 1e6;
% module_cable_kd = 5e3;
module_cable_pid(1) = PID(module_cable_u_max, module_cable_ep_max * len_error_scalar_module_cable, module_cable_ev_max * vel_error_scalar_module_cable, module_cable_kp, module_cable_kd);
module_cable_pid(2) = PID(module_cable_u_max, module_cable_ep_max * len_error_scalar_module_cable, module_cable_ev_max * vel_error_scalar_module_cable, module_cable_kp, module_cable_kd);
module_cable_pid(3) = PID(module_cable_u_max, module_cable_ep_max * len_error_scalar_module_cable, module_cable_ev_max * vel_error_scalar_module_cable, module_cable_kp, module_cable_kd);
module_cable_pid(4) = PID(module_cable_u_max, module_cable_ep_max * len_error_scalar_module_cable, module_cable_ev_max * vel_error_scalar_module_cable, module_cable_kp, module_cable_kd);
module_cable_pid(1).setUmin(fmin);
module_cable_pid(2).setUmin(fmin);
module_cable_pid(3).setUmin(fmin);
module_cable_pid(4).setUmin(fmin);
module_cable_pid(1).setGainCurve(positionLoopPower, velocityLoopPower);
module_cable_pid(2).setGainCurve(positionLoopPower, velocityLoopPower);
module_cable_pid(3).setGainCurve(positionLoopPower, velocityLoopPower);
module_cable_pid(4).setGainCurve(positionLoopPower, velocityLoopPower);
module_cable_pid(1).setCableLengthController();
module_cable_pid(2).setCableLengthController();
module_cable_pid(3).setCableLengthController();
module_cable_pid(4).setCableLengthController();

steering_inertia_gain = 1;
steering_u_max = 3e3;
steering_ep_max = 0.1;
steering_ev_max = 100.0;
steering_kp = steering_u_max / steering_ep_max;
steering_kd = steering_inertia_gain * damping_ratio_steering_motor * sqrt(steering_kp / steering_inertia_gain);
steering_motor_pid = PID(steering_u_max + steering_kd * steering_ev_max, steering_ep_max, steering_ev_max, steering_kp, steering_kd);
% steering_motor_pid.setCableLengthController();

linear_u_max = 2e4;
linear_ep_max = 0.01;
linear_ev_max = 0.5;
linear_kp = linear_u_max / linear_ep_max;
linear_kd = damping_ratio_linear_motor * sqrt(linear_kp);
linear_motor_pid(1) = PID(linear_u_max, linear_ep_max, linear_ev_max, linear_kp, linear_kd);
linear_motor_pid(2) = PID(linear_u_max, linear_ep_max, linear_ev_max, linear_kp, linear_kd);
% linear_motor_pid(1).setCableLengthController();
% linear_motor_pid(2).setCableLengthController();

thruster_gains = 5000.0 * ones(8, 1);
% thruster_gains = 0.0 * ones(8, 1);

%% setup simulation

% get the balancing starting pose
% var2 1.5*0.1 steering shaft cable attachment spread with 0.8 scaled hoisting cable motor placement

% crane and module cables use 0.01
pose = [1.08215911115172e-05; -5.12917804156128e-13; -0.00980999998428029; -1.17286953009825e-14; -5.23624275607442e-09; 3.85563683903891e-11; -3.96927324647810e-15; 5.55542577160217e-07; -3.87059905606800e-11; -2.70069858646592e-06; -4.22553562612795e-14; -0.00404906312970885; -7.78322113374218e-15; 5.19495496359785e-07; 4.09970396543864e-14; 3.37136166922785e-08; 2.49384185731063e-15; -8.77227456215430e-08; 8.81436715827661e-14; -0.00981000002424929; -3.42116570493888e-16; 6.31405317837469e-09; 1.52121422157401e-13];

% % crane and module cables use 0.001
% pose = [1.25900437321041e-06; 4.11498508030049e-15; -0.00981000000210941; 7.53659023428964e-16; -5.94195514332191e-10; -2.85978249490525e-12; 2.86436937004925e-15; 7.10443902807680e-08; 2.83606729440105e-12; -3.36178989142962e-07; -1.09859968035279e-14; -0.00404906313727403; -1.99082524720068e-15; 6.46663813133427e-08; 1.63934283601274e-14; 3.89654689136300e-09; -4.83165687571219e-16; -2.08701153716873e-08; -7.15720662808231e-15; -0.00981000004210120; 1.84815214118431e-16; 7.85534073639523e-10; 1.01723208794034e-14];

vel = zeros(size(pose));
q0_ctrl = 0 * pose;
q0_sim = pose;

%% stabilize the system

xTransMin = -0.3;
xTransMax = 0.3;
yTransMin = -0.2;
yTransMax = 0.2;
zTransMin = 0.0;
zTransMax = -3.0;

% xTransMin = 0.0;
% xTransMax = 0.0;
% yTransMin = 0.0;
% yTransMax = 0.0;
% zTransMin = 0.0;
% zTransMax = 0.0;

xStep = 0.05;
yStep = 0.05;
zStep = -0.1;

crane_hook_hoister_joint_idx_set = 1:6;
crane_hook_joint_idx_set = 7:8;
steering_joint_idx_set = 9;
hoisting_platform_joint_idx_set = 10:15;
linear_joint_idx_set = 16:17;
module_joint_idx_set = 18:23;

xLen = 1 + floor((xTransMax - xTransMin) / xStep);
yLen = 1 + floor((yTransMax - yTransMin) / yStep);
zLen = 1 + floor((zTransMax - zTransMin) / zStep);

% % point number based progress backup
% numPoints = xLen * yLen * zLen;
% ptCounter = 0;
% progressSavingFrequency = 0.005;
% progressSavingTickCount = round(numPoints * progressSavingFrequency);

% system time based progress backup
progressSavingTimeInterval = 1200; % 20 min
tic_stamp = tic;

data_file_path = ".\data\analysis_result\APAS\" + mdl_sim.robotName + ".mat";

calculate_map = false;

if (isfile(data_file_path))

    loaded_data = load(data_file_path);

    isConsistent = true;

    if xTransMin ~= loaded_data.xTransMin
        isConsistent = false;
    end

    if xTransMax ~= loaded_data.xTransMax
        isConsistent = false;
    end

    if xStep ~= loaded_data.xStep
        isConsistent = false;
    end

    if yTransMin ~= loaded_data.yTransMin
        isConsistent = false;
    end

    if yTransMax ~= loaded_data.yTransMax
        isConsistent = false;
    end

    if yStep ~= loaded_data.yStep
        isConsistent = false;
    end

    if zTransMin ~= loaded_data.zTransMin
        isConsistent = false;
    end

    if zTransMax ~= loaded_data.zTransMax
        isConsistent = false;
    end

    if zStep ~= loaded_data.zStep
        isConsistent = false;
    end

    if isConsistent
        calculate_map = true;
        result_processed_flag = loaded_data.result_processed_flag;
        result_valid_flag = loaded_data.result_valid_flag;
        result_trans_error = loaded_data.result_trans_error;
        result_attd_error = loaded_data.result_attd_error;
        result_joint_pose = loaded_data.result_joint_pose;
    else
        calculate_map = true;
        result_processed_flag = zeros(xLen, yLen, zLen);
        result_valid_flag = zeros(xLen, yLen, zLen);
        result_trans_error = zeros(xLen, yLen, zLen, 3);
        result_attd_error = zeros(xLen, yLen, zLen, 3);
        result_joint_pose = zeros(xLen, yLen, zLen, mdl_sim.numDofs);
    end

else
    calculate_map = true;
    result_processed_flag = zeros(xLen, yLen, zLen);
    result_valid_flag = zeros(xLen, yLen, zLen);
    result_trans_error = zeros(xLen, yLen, zLen, 3);
    result_attd_error = zeros(xLen, yLen, zLen, 3);
    result_joint_pose = zeros(xLen, yLen, zLen, mdl_sim.numDofs);
end

delta_t = 0.001;
t_travel = 10;
t_hold = 50;
t_max = 15;
termination_check_time_horizon = 15;
err_threshold_trans = 5e-4; % 0.5 mm
err_threshold_rot = 1e-3; % ~0.05 deg

if calculate_map

    for zIdx = 1:zLen
        z = zTransMin + (zIdx - 1) * zStep;

        for xIdx = 1:xLen
            x = xTransMin + (xIdx - 1) * xStep;

            for yIdx = 1:yLen
                y = yTransMin + (yIdx - 1) * yStep;

                z
                x
                y
                timeElapsedFromLastCheckpointInTheBeginningOfTheSim = toc(tic_stamp)

                if (~result_processed_flag(xIdx, yIdx, zIdx))

                    mdl_sim.update(q0_sim, zeros(size(q0_sim)), zeros(size(q0_sim)), zeros(size(q0_sim)));
                    module_pose_change_des = [x; y; z; 0.0];
                    refTraj = GenerateAPASTrajectory(module_pose_change_des, t_travel, t_hold, delta_t);

                    [res_valid, trans_err, attd_err_euler, pose] = APASTravelToSetpoint( ...
                        mdl_ctrl, ...
                        mdl_sim, ...
                        @APASVar2TaskSpaceUpdate, ...
                        @APASVar2StepModulePositionAndSteeringAngle, ...
                        q0_ctrl, ...
                        q0_sim, ...
                        refTraj, ...
                        steering_joint_idx_set, ...
                        linear_joint_idx_set, ...
                        crane_cable_pid, ...
                        module_cable_pid, ...
                        hoisting_cable_pid, ...
                        steering_motor_pid, ...
                        linear_motor_pid, ...
                        thruster_gains, ...
                        t_travel, ...
                        err_threshold_trans, ...
                        err_threshold_rot, ...
                        termination_check_time_horizon);

                    % [res_valid, trans_err, attd_err_euler, pose] = APASStabilizeAtSetpoint( ...
                    %     mdl_ctrl, ...
                    %     mdl_sim, ...
                    %     @APASVar2TaskSpaceUpdate, ...
                    %     @APASVar2StepModulePositionAndSteeringAngle, ...
                    %     q0_ctrl, ...
                    %     q0_sim, ...
                    %     module_pose_change_des, ...
                    %     steering_joint_idx_set, ...
                    %     linear_joint_idx_set, ...
                    %     crane_cable_pid, ...
                    %     module_cable_pid, ...
                    %     hoisting_cable_pid, ...
                    %     steering_motor_pid, ...
                    %     linear_motor_pid, ...
                    %     thruster_gains, ...
                    %     delta_t, ...
                    %     t_max, ...
                    %     err_threshold_trans, ...
                    %     err_threshold_rot, ...
                    %     termination_check_time_horizon);

                    trans_err
                    attd_err_euler
                    result_processed_flag(xIdx, yIdx, zIdx) = true;
                    result_valid_flag(xIdx, yIdx, zIdx) = res_valid;
                    result_trans_error(xIdx, yIdx, zIdx, :) = trans_err;
                    result_attd_error(xIdx, yIdx, zIdx, :) = attd_err_euler;
                    result_joint_pose(xIdx, yIdx, zIdx, :) = pose;
                end

                timeElapsedFromLastCheckpointAtTheEndOfTheSim = toc(tic_stamp)

                % % point count based progress saving
                % ptCounter = ptCounter + 1;
                % if mod(ptCounter, progressSavingTickCount) == 0
                %     save(data_file_path, "result_*", "xTransMin", "xTransMax", "xStep", "yTransMin", "yTransMax", "yStep", "zTransMin", "zTransMax", "zStep");
                % end

                % system time based progress saving
                if toc(tic_stamp) > progressSavingTimeInterval
                    save(data_file_path, "result_*", "xTransMin", "xTransMax", "xStep", "yTransMin", "yTransMax", "yStep", "zTransMin", "zTransMax", "zStep");
                    tic_stamp = tic;
                end

            end

        end

        save(data_file_path, "result_*", "xTransMin", "xTransMax", "xStep", "yTransMin", "yTransMax", "yStep", "zTransMin", "zTransMax", "zStep");

    end

    save(data_file_path, "result_*", "xTransMin", "xTransMax", "xStep", "yTransMin", "yTransMax", "yStep", "zTransMin", "zTransMax", "zStep");
end
