function [steering_jnt, hoisting_cable_l, horizontal_trans, vertical_trans] = APASVar1StepModulePositionAndSteeringAngle(mdl, q0, module_pose_change)

    % get relevant indices
    if strcmp(mdl.robotName, 'APAS_var1')
        hoisting_frame_link_idx = 4;
        horizontal_frame_link_idx = 5;
        vertical_frame_link_idx = 6;
        module_link_idx = 7;
        hoisting_platform_jointX_idx = 10;
        hoisting_platform_jointY_idx = 11;
        hoisting_platform_jointZ_idx = 12;
        horizontal_trans_idx = 16;
        vertical_trans_idx = 17;
        steering_jnt_idx = 9;
    elseif strcmp(mdl.robotName, 'APASR_var1')
        hoisting_frame_link_idx = 17;
        horizontal_frame_link_idx = 18;
        vertical_frame_link_idx = 19;
        module_link_idx = 20;
        hoisting_platform_jointX_idx = 22;
        hoisting_platform_jointY_idx = 23;
        hoisting_platform_jointZ_idx = 24;
        horizontal_trans_idx = 28;
        vertical_trans_idx = 29;
        steering_jnt_idx = 9;
    else
        hoisting_frame_link_idx = mdl.getLinkIndex('hoisting_platform');
        horizontal_frame_link_idx = mdl.getLinkIndex('horizontal_moving_platform');
        vertical_frame_link_idx = mdl.getLinkIndex('vertical_moving_platform');
        module_link_idx = mdl.getLinkIndex('module');
        steering_shaft_idx = mdl.getLinkIndex('steering_shaft');
        hoisting_platform_jointX_idx = mdl.dof_interval_links(1, hoisting_frame_link_idx);
        hoisting_platform_jointY_idx = hoisting_platform_jointX_idx + 1;
        hoisting_platform_jointZ_idx = hoisting_platform_jointX_idx + 2;
        horizontal_trans_idx = mdl.dof_interval_links(2, horizontal_frame_link_idx);
        vertical_trans_idx = mdl.dof_interval_links(2, vertical_frame_link_idx);
        steering_jnt_idx = mdl.dof_interval_links(2, steering_shaft_idx);
    end

    % get mass properties
    mass_hoisting_frame = mdl.m(hoisting_frame_link_idx);
    mass_horizontal_frame = mdl.m(horizontal_frame_link_idx);
    mass_vertical_frame = mdl.m(vertical_frame_link_idx);
    mass_module = mdl.m(module_link_idx);
    horizontal_moving_mass = mass_horizontal_frame + mass_vertical_frame + mass_module;
    vertical_moving_mass = mass_vertical_frame + mass_module;
    total_mass = mass_hoisting_frame + mass_horizontal_frame + mass_vertical_frame + mass_module;

    % get desired steering joint position
    steering_jnt = module_pose_change(4) + q0(steering_jnt_idx);

    % convert module translation (position change) defined in world frame to local frame
    trans = QuaternionToRotationMatrix(QuaternionRZ(steering_jnt)) * [module_pose_change(1); module_pose_change(2); module_pose_change(3)];

    % calculate desired xy table position change
    horizontal_trans_step = trans(1) * total_mass / (total_mass - horizontal_moving_mass);
    vertical_trans_step = trans(2) * total_mass / (total_mass - vertical_moving_mass);

    % calculate desired xy table position
    horizontal_trans = q0(horizontal_trans_idx) + horizontal_trans_step;
    vertical_trans = q0(vertical_trans_idx) + vertical_trans_step;

    % calculate hoisting platform jpos change
    hoisting_platform_dx = trans(1) - horizontal_trans_step;
    hoisting_platform_dy = trans(2) - vertical_trans_step;
    hoisting_platform_dz = trans(3);

    q = q0;
    q(hoisting_platform_jointX_idx) = q(hoisting_platform_jointX_idx) + hoisting_platform_dx;
    q(hoisting_platform_jointY_idx) = q(hoisting_platform_jointY_idx) + hoisting_platform_dy;
    q(hoisting_platform_jointZ_idx) = q(hoisting_platform_jointZ_idx) + hoisting_platform_dz;

    mdl.update(q, zeros(size(q)), zeros(size(q)), zeros(size(q)));

    hoisting_cable_l = [mdl.cableLength(3);
                        mdl.cableLength(4);
                        mdl.cableLength(5);
                        mdl.cableLength(6)];
end
