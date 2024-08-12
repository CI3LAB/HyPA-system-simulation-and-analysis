clc; clear; close all;
%% build the model
def_apas_var2;
% def_apas_realistic;
mdl_ctrl = RigidBodySystem(model_def);

mdl_sim = RigidBodySystem(model_def);

%% load data

data_file_path = ".\data\analysis_result\APAS\" + mdl_sim.robotName + ".mat";

% selected_position_idx = [23; 15; 1]; % [xIdx; yIdx; zIdx]
selected_position_idx = [11; 9; 1]; % [xIdx; yIdx; zIdx]
data_valid = true;

% define the desired range
xIdxOffset = 1;
yIdxOffset = 0;
zIdxOffset = 0;
xOneSideLength = 5;
yOneSideLength = 4;
zOneSideLength = 6;

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

    if xLen < 1 + xIdxOffset + 2 * xOneSideLength
        data_valid = false;
    else
        xLen = 1 + 2 * xOneSideLength;
    end

    if yLen < 1 + yIdxOffset + 2 * yOneSideLength
        data_valid = false;
    else
        yLen = 1 + 2 * yOneSideLength;
    end

    if zLen < 1 + zIdxOffset + zOneSideLength
        data_valid = false;
    else
        zLen = 1 + zOneSideLength;
    end

    xRange = loaded_data.xTransMin - loaded_data.xStep + loaded_data.xStep * (xIdxOffset + (1:xLen));
    yRange = loaded_data.yTransMin - loaded_data.yStep + loaded_data.yStep * (yIdxOffset + (1:yLen));
    zRange = loaded_data.zTransMin - loaded_data.zStep + loaded_data.zStep * (zIdxOffset + (1:zLen));

    selectedX = xRange(xIdxSelected)
    selectedY = yRange(yIdxSelected)
    selectedZ = zRange(zIdxSelected)

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

    trans_err = zeros(xLen, yLen, zLen);
    attd_err = zeros(xLen, yLen, zLen);
    trans_h_err = zeros(xLen, yLen, zLen);
    trans_v_err = zeros(xLen, yLen, zLen);
    trans_x_err = zeros(xLen, yLen, zLen);
    trans_y_err = zeros(xLen, yLen, zLen);
    trans_z_err = zeros(xLen, yLen, zLen);
    attd_x_err = zeros(xLen, yLen, zLen);
    attd_y_err = zeros(xLen, yLen, zLen);
    attd_z_err = zeros(xLen, yLen, zLen);

    x_vec = zeros(xLen * yLen * zLen, 1);
    y_vec = zeros(xLen * yLen * zLen, 1);
    z_vec = zeros(xLen * yLen * zLen, 1);
    trans_err_vec = zeros(xLen * yLen * zLen, 1);
    attd_err_vec = zeros(xLen * yLen * zLen, 1);
    trans_h_err_vec = zeros(xLen * yLen * zLen, 1);
    trans_v_err_vec = zeros(xLen * yLen * zLen, 1);
    trans_x_err_vec = zeros(xLen * yLen * zLen, 1);
    trans_y_err_vec = zeros(xLen * yLen * zLen, 1);
    trans_z_err_vec = zeros(xLen * yLen * zLen, 1);
    attd_x_err_vec = zeros(xLen * yLen * zLen, 1);
    attd_y_err_vec = zeros(xLen * yLen * zLen, 1);
    attd_z_err_vec = zeros(xLen * yLen * zLen, 1);

    pt_idx = 0;

    trans_err_offset = reshape( ...
        loaded_data.result_trans_error( ...
        xIdxOffset + xOneSideLength + 1, ...
        yIdxOffset + yOneSideLength + 1, ...
        zIdxOffset + 1, ...
        :), ...
        [3, 1]);
    attd_err_offset = reshape( ...
        loaded_data.result_attd_error( ...
        xIdxOffset + xOneSideLength + 1, ...
        yIdxOffset + yOneSideLength + 1, ...
        zIdxOffset + 1, ...
        :), ...
        [3, 1]);

    for i = 1:xLen

        for j = 1:yLen

            for k = 1:zLen
                pt_idx = pt_idx + 1;
                x_vec(pt_idx) = xRange(i);
                y_vec(pt_idx) = yRange(j);
                z_vec(pt_idx) = zRange(k);

                if loaded_data.result_valid_flag(xIdxOffset + i, yIdxOffset + j, zIdxOffset + k) == false
                    trans_err(i, j, k) = NaN;
                    trans_h_err(i, j, k) = NaN;
                    trans_v_err(i, j, k) = NaN;
                    trans_x_err(i, j, k) = NaN;
                    trans_y_err(i, j, k) = NaN;
                    trans_z_err(i, j, k) = NaN;
                    attd_err(i, j, k) = NaN;
                    attd_x_err(i, j, k) = NaN;
                    attd_y_err(i, j, k) = NaN;
                    attd_z_err(i, j, k) = NaN;
                    trans_err_vec(pt_idx) = NaN;
                    trans_h_err_vec(pt_idx) = NaN;
                    trans_v_err_vec(pt_idx) = NaN;
                    trans_x_err_vec(pt_idx) = NaN;
                    trans_y_err_vec(pt_idx) = NaN;
                    trans_z_err_vec(pt_idx) = NaN;
                    attd_err_vec(pt_idx) = NaN;
                    attd_x_err_vec(pt_idx) = NaN;
                    attd_y_err_vec(pt_idx) = NaN;
                    attd_z_err_vec(pt_idx) = NaN;
                else
                    trans_err_selected = reshape(loaded_data.result_trans_error(xIdxOffset + i, yIdxOffset + j, zIdxOffset + k, :), [3, 1]) - trans_err_offset;
                    attd_err_selected = reshape(loaded_data.result_attd_error(xIdxOffset + i, yIdxOffset + j, zIdxOffset + k, :), [3, 1]) - attd_err_offset;
                    trans_err(i, j, k) = norm(trans_err_selected);
                    trans_h_err(i, j, k) = norm(trans_err_selected(1:2));
                    trans_v_err(i, j, k) = norm(trans_err_selected(3));
                    trans_x_err(i, j, k) = trans_err_selected(1);
                    trans_y_err(i, j, k) = trans_err_selected(2);
                    trans_z_err(i, j, k) = trans_err_selected(3);
                    attd_err(i, j, k) = norm(attd_err_selected, 1);
                    attd_x_err(i, j, k) = attd_err_selected(1);
                    attd_y_err(i, j, k) = attd_err_selected(2);
                    attd_z_err(i, j, k) = attd_err_selected(3);

                    trans_err_vec(pt_idx) = norm(trans_err_selected);
                    trans_h_err_vec(pt_idx) = norm(trans_err_selected(1:2));
                    trans_v_err_vec(pt_idx) = norm(trans_err_selected(3));
                    trans_x_err_vec(pt_idx) = trans_err_selected(1);
                    trans_y_err_vec(pt_idx) = trans_err_selected(2);
                    trans_z_err_vec(pt_idx) = trans_err_selected(3);
                    attd_err_vec(pt_idx) = norm(attd_err_selected, 1);
                    attd_x_err_vec(pt_idx) = attd_err_selected(1);
                    attd_y_err_vec(pt_idx) = attd_err_selected(2);
                    attd_z_err_vec(pt_idx) = attd_err_selected(3);
                end

            end

        end

    end

    % do the plotting

    marker_size = 20.0;

    font_name = 'Times';
    general_font_size = 14;
    axis_tick_font_size = 10;

    LineStyleLib = {'-', '--', ':', '-.'};
    bg = {'k', 'w'};
    colors = distinguishable_colors(8);

    scatter_view_angle = [-27.239586997021597, 83.854204234755599];
    scatter_view_angle = [-25.0, 83.0];

    figure;
    error = 1000 * (trans_err_vec');
    markerSize = marker_size * ones(size(error));
    scatter3(x_vec, y_vec, z_vec, markerSize, error, 'filled');
    colorbar;
    grid on;
    xlabel('X (m)', 'FontSize', general_font_size, 'FontName', font_name);
    ylabel('Y (m)', 'FontSize', general_font_size, 'FontName', font_name);
    zlabel('Z (m)', 'FontSize', general_font_size, 'FontName', font_name);
    title('Translation Error (mm)');
    figXLim = xlim;
    figYLim = ylim;
    figZLim = zlim;
    pbaspect([figXLim(2) - figXLim(1) figYLim(2) - figYLim(1) figZLim(2) - figZLim(1)]);
    view(scatter_view_angle);

    figure;
    error = 1000 * (trans_h_err_vec');
    markerSize = marker_size * ones(size(error));
    scatter3(x_vec, y_vec, z_vec, markerSize, error, 'filled');
    colorbar;
    grid on;
    xlabel('X (m)', 'FontSize', general_font_size, 'FontName', font_name);
    ylabel('Y (m)', 'FontSize', general_font_size, 'FontName', font_name);
    zlabel('Z (m)', 'FontSize', general_font_size, 'FontName', font_name);
    title('Horizontal Translation Error (mm)');
    figXLim = xlim;
    figYLim = ylim;
    figZLim = zlim;
    pbaspect([figXLim(2) - figXLim(1) figYLim(2) - figYLim(1) figZLim(2) - figZLim(1)]);
    view(scatter_view_angle);

    figure;
    error = 1000 * (trans_v_err_vec');
    markerSize = marker_size * ones(size(error));
    scatter3(x_vec, y_vec, z_vec, markerSize, error, 'filled');
    colorbar;
    grid on;
    xlabel('X (m)', 'FontSize', general_font_size, 'FontName', font_name);
    ylabel('Y (m)', 'FontSize', general_font_size, 'FontName', font_name);
    zlabel('Z (m)', 'FontSize', general_font_size, 'FontName', font_name);
    title('Vertical Translation Error (mm)');
    figXLim = xlim;
    figYLim = ylim;
    figZLim = zlim;
    pbaspect([figXLim(2) - figXLim(1) figYLim(2) - figYLim(1) figZLim(2) - figZLim(1)]);
    view(scatter_view_angle);

    figure;
    error = rad2deg(attd_err_vec');
    markerSize = marker_size * ones(size(error));
    scatter3(x_vec, y_vec, z_vec, markerSize, error, 'filled');
    colorbar;
    grid on;
    xlabel('X (m)', 'FontSize', general_font_size, 'FontName', font_name);
    ylabel('Y (m)', 'FontSize', general_font_size, 'FontName', font_name);
    zlabel('Z (m)', 'FontSize', general_font_size, 'FontName', font_name);
    title('Attitude Error (deg)');
    figXLim = xlim;
    figYLim = ylim;
    figZLim = zlim;
    pbaspect([figXLim(2) - figXLim(1) figYLim(2) - figYLim(1) figZLim(2) - figZLim(1)]);
    view(scatter_view_angle);

end
