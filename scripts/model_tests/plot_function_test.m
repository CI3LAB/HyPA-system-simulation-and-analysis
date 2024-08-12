clc; clear; close all;
% def_optimized_omnidirectional_multirotor
% def_quadrotor;
% def_reconfigurable_quadrotor
% def_hexarotor; % model_def.robotName = 'Hexarotor'
% def_fully_restrained_multirotor; % model_def.robotName = 'Fully-restrained multirotor'
% def_BMArm;
% def_8S_neck;
% def_8S_neck_CASPR;
% def_cable_thruster_hybrid_crane_scheme2;
% def_cable_thruster_hybrid_crane_scheme3;
% def_TwoBranchArm_CASPR;
% def_spider_arm;
% def_planar_XZ;
% def_example_planar_2R_XZ;
% def_example_spatial;
% def_4cable_suspended_cdr;
% def_inverted_cable_hoister;
% def_Lab_Prototype;
% def_inverted_cable_hoister_var1;
% def_inverted_cable_hoister_var2;
% def_SJTUSnake_CASPR;
% def_snakearm12;
% def_cable_hoister;
% def_apas;
% def_apas_var0;
def_apas_var1;
% def_apas_var1_touched_down;
% def_apas_var2;
% def_reduced_apas_var2;
% def_apas_realistic;
% def_apas_s;
% def_cable_hoister;
% def_apas2_var0;
% def_apas2_var1;
% def_apas2_var2;
% def_apas2;
% def_apas3;
mdl = RigidBodySystem(model_def);

if strcmp(mdl.robotName, 'HybridCraneScheme2')
    x0 = [0; 0; 0; 0; 0; 1; 0; 0; 0];
    mdl.update(x0, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    plot_axis = 1 * [-13 13 -13 13 -25 1];
    view_angle = [45 45];
elseif strcmp(mdl.robotName, 'InvertedCableHoister')
    x0 = [0; 0; -5; 0; 0; 0];
    mdl.update(x0, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    plot_axis = 1 * [-8 8 -5 5 -10 1];
    view_angle = [45 45];
    view_angle = [45 45];
elseif strcmp(mdl.robotName, 'HybridCraneScheme3')
    x0 = [0; 0; 1; 0; 0; 0];
    mdl.update(x0, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    plot_axis = 1 * [-13 13 -13 13 -25 1];
    view_angle = [45 45];
elseif strcmp(mdl.robotName, 'SnakeArm12')
    tmp = [1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0];
    pose = 1 * tmp;
    pose = 0.7 * ones(mdl.numDofs, 1);
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.3 0.3 -0.3 0.3 -0.1 2.2];
    plot_axis = 0.7 * [-1 1 -1 1 -1 1];
    view_angle = [45 0];
elseif strcmp(mdl.robotName, 'APAS')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    %     hoisting_frame_spatial_joint(3) = 0.5;
    horizontal_linear_joint = 0.4;
    vertical_linear_joint = -0.3;
    horizontal_linear_joint = 0.0;
    vertical_linear_joint = -0.0;
    module_spatial_joint = zeros(6, 1);

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                horizontal_linear_joint; ...
                vertical_linear_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 2.2 0.0 2.0 2.9 10.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APAS_var0') || strcmp(mdl.robotName, 'APAS_var1') || strcmp(mdl.robotName, 'APAS_var2')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    %     hoisting_frame_spatial_joint(3) = 0.5;
    horizontal_linear_joint = 0.4;
    vertical_linear_joint = -0.3;
    horizontal_linear_joint = 0.0;
    vertical_linear_joint = -0.0;
    module_spatial_joint = zeros(6, 1);

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                horizontal_linear_joint; ...
                vertical_linear_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 10.2 -0.2 10.2 -0.1 60.1];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'ReducedAPAS_var0') || strcmp(mdl.robotName, 'ReducedAPAS_var1') || strcmp(mdl.robotName, 'ReducedAPAS_var2')
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    horizontal_linear_joint = 0.0;
    vertical_linear_joint = -0.0;
    module_spatial_joint = zeros(6, 1);

    pose = [ ...
                hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                horizontal_linear_joint; ...
                vertical_linear_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 10.2 -0.2 10.2 -0.1 16.1];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APAS2')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    module_spatial_joint = zeros(6, 1);

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 2.2 0.0 2.0 2.9 10.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APAS2_var0')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    module_spatial_joint = zeros(6, 1);

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 2.2 0.0 2.0 2.9 10.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APAS2_var1')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    module_spatial_joint = zeros(6, 1);

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 2.2 0.0 2.0 2.9 10.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APAS2_var2')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    module_spatial_joint = zeros(6, 1);

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                hoisting_frame_spatial_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 2.2 0.0 2.0 2.9 10.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APAS3')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    module_spatial_joint = zeros(6, 1);

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 2.2 0.0 2.0 2.9 10.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APAS_S')
    clear model_def
    def_apas;
    model_def = RemoveThrustersCraneCablesAndCraneHook(model_def);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    %     hoisting_frame_spatial_joint(3) = 0.5;
    horizontal_linear_joint = 0.4;
    vertical_linear_joint = -0.3;
    horizontal_linear_joint = 0.0;
    vertical_linear_joint = -0.0;
    module_spatial_joint = zeros(6, 1);

    pose = [hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                horizontal_linear_joint; ...
                vertical_linear_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 2.2 0.0 2.0 2.9 10.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APASR')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    cable1_universal_joint = [-1.0; 0.5; 0.5];
    cable2_universal_joint = [-1.0; -0.5; -0.5];
    cable3_universal_joint = [1.0; -0.5; -0.5];
    cable4_universal_joint = [1.0; 0.5; 0.5];

    hoisting_frame_spatial_joint = zeros(6, 1);
    %     hoisting_frame_spatial_joint(3) = 0.5;
    horizontal_linear_joint = 0.4;
    vertical_linear_joint = -0.3;
    horizontal_linear_joint = 0.0;
    vertical_linear_joint = -0.0;
    module_spatial_joint = zeros(6, 1);

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                steering_joint; ...
                cable1_universal_joint; ...
                cable2_universal_joint; ...
                cable3_universal_joint; ...
                cable4_universal_joint; ...
                hoisting_frame_spatial_joint; ...
                horizontal_linear_joint; ...
                vertical_linear_joint; ...
                module_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 2.2 0.0 2.0 1.0 8.0];
    plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'CableHoister')
    hook_spatial_joint = [4.0; 0; 3; 0; -deg2rad(30); 0];

    hook_spatial_joint = [0.0; 0.0; 0.0; 0; -deg2rad(0); 0];

    pose = [hook_spatial_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-6.0 6.0 -3.0 3.0 -0.1 15.0];
    view_angle = [30 20];
elseif strcmp(mdl.robotName, 'APAS_var1_touched_down')
    crane_hook_spatial_joint = zeros(6, 1);
    hook_universal_joint = [0.0; 0.0];
    steering_joint = 0;
    hoisting_frame_spatial_joint = zeros(6, 1);
    linear_x_position = 3.0;
    linear_y_position = -1.2;
    hoisting_frame_spatial_joint(1) = -linear_x_position;
    hoisting_frame_spatial_joint(2) = -linear_y_position;
    horizontal_linear_joint = linear_x_position;
    vertical_linear_joint = linear_y_position;

    pose = [crane_hook_spatial_joint; ...
                hook_universal_joint; ...
                steering_joint; ...
                hoisting_frame_spatial_joint; ...
                horizontal_linear_joint; ...
                vertical_linear_joint ...
            ];
    mdl.update(pose, zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    % mdl.update(zeros(mdl.dimPosition,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1),zeros(mdl.numDofs,1));
    plot_axis = [-0.2 10.2 -0.2 10.2 -0.1 60.1];
    %     plot_axis = [-0.2 2.2 0.0 2.0 0.0 6.0];
    %     plot_axis = [-0.2 2.2 0.0 2.0 1.1 4.3];
    %     plot_axis = 0.7*[-1 1 -1 1 -1 1];
    view_angle = [30 20];
else
    mdl.update(zeros(mdl.dimPosition, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1), zeros(mdl.numDofs, 1));
    plot_axis = 0.5 * [-1 1 -1 1 -1 1];
    view_angle = [45 45];
end

% view_angle = [1 1 1];
fig_handle = figure();
axis_handle = gca;
transparencyRatio = 0.5;
ModelVisualization.PlotFrame(mdl, plot_axis, view_angle, fig_handle, axis_handle, transparencyRatio);
% ModelVisualization.PlotFrame(mdl, plot_axis, view_angle);
