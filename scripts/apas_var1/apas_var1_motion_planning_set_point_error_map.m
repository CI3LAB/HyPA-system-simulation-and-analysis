clc; clear; close all;
%% build the model
def_apas_var1;
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

if strcmp(mdl_sim.robotName, 'APAS_var1')
    crane_cable_load = 10 * sum(mdl_sim.m(:)) / 2.0
    hoisting_cable_load = 10 * sum(mdl_sim.m(4:end)) / 4.0
    module_cable_load = 10 * sum(mdl_sim.m(end)) / 4.0
elseif strcmp(mdl_sim.robotName, 'APASR_var1')
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

thruster_gains = 8000.0 * ones(8, 1);
% thruster_gains = 0.0 * ones(8, 1);

%% setup simulation

% get the balancing starting pose
% crane and module cables use 0.01
% pose = [0.000332184716297000; -7.85375398746141e-15; -0.00980999319627601; -1.55952845847426e-15; -2.17567237328855e-07; 3.06481037510074e-14; 3.12851179106990e-15; -0.0519417037259903; -3.63860746328292e-14; 0.0691429535424710; -1.62046411935668e-14; -0.0151071448802948; -3.35359289617379e-15; 0.0520404792653995; 5.87244341545912e-15; -0.403322812640412; -5.69708219798497e-17; 0.000235926039592410; -1.50722035270033e-16; -0.00980998188867932; 6.99207082086250e-17; -1.58808808821948e-07; 2.38114766175602e-17];
pose = [-2.35846646619907e-10; 1.46446873897852e-15; -0.00980999999479198; 5.87411997109942e-16; 1.53695574573421e-13; -3.48159604081899e-15; -7.84094576873428e-15; 2.34556206390728e-12; 8.43354687461575e-15; 1.17373313221636e-12; 3.77554756202113e-14; -0.00324968016812233; 6.90009649298614e-15; -2.29543034144879e-13; -4.94979424827989e-15; -9.79478382597999e-13; 1.67232446389078e-16; -2.95463765901634e-11; 2.08018258462742e-15; -0.00981000003468921; 3.07994538535395e-17; -1.92605402019783e-14; -1.68916924600482e-17];
% % crane and module cables use 0.001
% pose = [6.24433464451609e-10; -7.46149141970908e-15; -0.000981000000211963; 6.35068395875064e-15; -3.54296680817619e-14; -5.46285022847205e-15; -4.00824258097226e-15; -8.32672192011676e-12; 5.10906237200305e-15; -2.30041512766634e-12; -9.72981923100308e-15; -0.00324968017043639; -1.50617422174813e-15; 4.51335940033625e-13; 3.47455898586055e-16; 2.80168050753972e-12; -1.08447942805860e-16; 8.45211894944571e-11; -5.68222292803793e-15; -0.000981000004226841; -3.66873273686813e-17; 4.27140964131512e-15; -8.74447527870256e-18];
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

xLen = 1 + floor((xTransMax - xTransMin) / (0.99999 * xStep));
yLen = 1 + floor((yTransMax - yTransMin) / (0.99999 * yStep));
zLen = 1 + floor((zTransMax - zTransMin) / (0.99999 * zStep));

% % point number based progress backup
% numPoints = xLen * yLen * zLen;
% ptCounter = 0;
% progressSavingFrequency = 0.005;
% progressSavingTickCount = round(numPoints * progressSavingFrequency);

% system time based progress backup
progressSavingTimeInterval = 600; % 10 min
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
                        @APASVar1TaskSpaceUpdate, ...
                        @APASVar1StepModulePositionAndSteeringAngle, ...
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
                    %     @APASVar1TaskSpaceUpdate, ...
                    %     @APASVar1StepModulePositionAndSteeringAngle, ...
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
