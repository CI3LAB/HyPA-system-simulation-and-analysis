clc; clear; close all;
%% build the model
def_apas_var0;
% def_apas_realistic;
mdl_ctrl = RigidBodySystem(model_def);

mdl_sim = RigidBodySystem(model_def);

%% load data

data_file_path = ".\data\analysis_result\APAS\" + mdl_sim.robotName + ".mat";

selected_position_idx = [1; 1; 1]; % [xIdx; yIdx; zIdx]
data_valid = true;

xLen = 0;
yLen = 0;
zLen = 0;
xIdxSelected = selected_position_idx(1);
yIdxSelected = selected_position_idx(2);
zIdxSelected = selected_position_idx(3);
xRange = [];
yRange = [];
zRange = [];

if (isfile(data_file_path))

    loaded_data = load(data_file_path);

    xLen = size(loaded_data.result_valid_flag, 1);
    yLen = size(loaded_data.result_valid_flag, 2);
    zLen = size(loaded_data.result_valid_flag, 3);

    xRange = loaded_data.xTransMin - loaded_data.xStep + loaded_data.xStep * (1:xLen);
    yRange = loaded_data.yTransMin - loaded_data.yStep + loaded_data.yStep * (1:yLen);
    zRange = loaded_data.zTransMin - loaded_data.zStep + loaded_data.zStep * (1:zLen);

    if xIdxSelected < 1 || xIdxSelected > xLen
        data_valid = false;
    end

    if yIdxSelected < 1 || yIdxSelected > yLen
        data_valid = false;
    end

    if zIdxSelected < 1 || zIdxSelected > zLen
        data_valid = false;
    end

    % check if the selected z-plane has all the data processed
    if sum(loaded_data.result_processed_flag(:, :, zIdxSelected) == false) > 0
        data_valid = false;
    end

else
    data_valid = true;
end

%%

if data_valid
    % plot the selected z-plane result

    trans_err = zeros(xLen, yLen);
    attd_err = zeros(xLen, yLen);

    for i = 1:xLen

        for j = 1:yLen

            if loaded_data.result_valid_flag(i, j, zIdxSelected) == false
                trans_err(i, j) = NaN;
                attd_err(i, j) = NaN;
            else
                trans_err_selected = reshape(loaded_data.result_trans_error(i, j, zIdxSelected, :), [3, 1]);
                attd_err_selected = reshape(loaded_data.result_attd_error(i, j, zIdxSelected, :), [3, 1]);
                trans_err(i, j) = norm(trans_err_selected);
                attd_err(i, j) = norm(attd_err_selected, 1);
            end

        end

    end

    % do the plotting

    font_name = 'Times';
    general_font_size = 14;
    axis_tick_font_size = 10;

    LineStyleLib = {'-', '--', ':', '-.'};
    bg = {'k', 'w'};
    colors = distinguishable_colors(8);

    figure;
    [X, Y] = meshgrid(xRange, yRange);
    Z = trans_err';
    hp = surf(X, Y, Z);
    colorbar;
    grid on;
    xlabel('X (m)', 'FontSize', general_font_size, 'FontName', font_name);
    ylabel('Y (m)', 'FontSize', general_font_size, 'FontName', font_name);
    zlabel('Error (m)', 'FontSize', general_font_size, 'FontName', font_name);
    title('Translation Error');

    figure;
    [X, Y] = meshgrid(xRange, yRange);
    Z = attd_err';
    hp = surf(X, Y, Z);
    colorbar;
    grid on;
    xlabel('X (m)', 'FontSize', general_font_size, 'FontName', font_name);
    ylabel('Y (m)', 'FontSize', general_font_size, 'FontName', font_name);
    zlabel('Error (rad)', 'FontSize', general_font_size, 'FontName', font_name);
    title('Attitude Error');
end
