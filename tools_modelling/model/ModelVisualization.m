classdef ModelVisualization

properties (SetAccess = private)
    mdl
end
properties (Constant)
    MOVIE_DEFAULT_WIDTH = 700;
    MOVIE_DEFAULT_HEIGHT = 700;
end

methods
    function mv = ModelVisualization(mdl)
        mv.mdl = mdl;        
    end
end

methods (Static)

    % function that plots the robot frame. plot_axis represents the axis limits and view_angle represents the view angle as the name suggests
    %   This function is converted from MotionSimulator.PlotFrame() in CASPR
    function PlotFrame(mdl, plot_axis, view_angle, fig_handle, axis_handle, transparencyRatio, cable_color_vec)
        % color code: 
        colorCode_black = [0, 0, 0];
        colorCode_red = [1, 0, 0];
        colorCode_green = [0, 1, 0];
        colorCode_blue = [0, 0, 1];
        colorCode_magenta = [1, 0, 1];
        colorCode_cyan = [0, 1, 1];
        
        if nargin < 4 || isempty(fig_handle)
            figure;
        else
            figure(fig_handle);
        end
        if nargin < 5 || isempty(axis_handle)
            clf;
            ax = gca;
        else
            ax = axis_handle;
            axes(ax);
        end
        if nargin < 6
            transparencyRatio = 1.0;
        end
        if nargin < 7
            cable_color_vec = repmat(colorCode_red, mdl.numCables, 1);
        end
        
        hold on;
        grid on;
        xlabel('x (m)');
        ylabel('y (m)');
        zlabel('z (m)');
        
        % plot link
        for k = 1:mdl.numLinks
            R = QuaternionToRotationMatrix(mdl.quat_links(:,k))';
            r_OP0 = R*mdl.OP_links(:,k);
            r_OG0 = r_OP0 + R*mdl.CoM(:,k);
            body = [];
            VLk = mdl.VL{k};
            for i = 1:length(VLk)
                body = [body, r_OP0 + R*Polyhedron(VLk{i})];
            end
            plot3(ax,r_OP0(1), r_OP0(2), r_OP0(3), 'Color', [colorCode_black, transparencyRatio*1.0], 'Marker', 'o', 'LineWidth', 2);
            plot3(ax,r_OG0(1), r_OG0(2), r_OG0(3), 'Color', [colorCode_blue, transparencyRatio*1.0], 'Marker', 'o', 'LineWidth', 2);
            for i = 1:length(body)
                if body(i).isFullDim
                    body(i).plot('LineColor', 'b', 'LineWidth', 0.5, 'Color', 'k', 'alpha', transparencyRatio*0.3, 'edgealpha',transparencyRatio*0.8);
                else
                    body(i).plot('LineColor', 'k', 'LineWidth', 3, 'Color', 'k', 'alpha', transparencyRatio*0.1);
                end
            end
        end

        % plot cable
        % step 1: represent all the attachment locations in the base frame
        attachment_loc_0 = zeros(size(mdl.cable_data_attachments));
        num_attachment = size(mdl.cable_data_attachments,2);
        for k = 0:mdl.numLinks
            if k == 0
                R = eye(3);
                r_OP0 = [0; 0; 0];
            else
                R = QuaternionToRotationMatrix(mdl.quat_links(:,k))';
                r_OP0 = R*mdl.OP_links(:,k);
            end
            for i = 1:num_attachment
                if mdl.cable_data_attachment_link_indices(i) == k
                    attachment_loc_0(:,i) = r_OP0 + R*mdl.cable_data_attachments(:,i);
                end
            end
        end
        %step 2: plot each cable
        for k = 1:mdl.numCables
            attachment_set_k = mdl.cables{k};
            prev_attachment = attachment_loc_0(:,attachment_set_k(1));
            for i = 2:length(attachment_set_k)
                current_attachment = attachment_loc_0(:,attachment_set_k(i));
                plot3(ax, [prev_attachment(1) current_attachment(1)], [prev_attachment(2) current_attachment(2)], [prev_attachment(3) current_attachment(3)], 'Color', [cable_color_vec(k, :), transparencyRatio*1.0], 'LineWidth', 1);
                prev_attachment = current_attachment;
            end
            first_attachment = attachment_loc_0(:,attachment_set_k(1));
            plot3(ax, first_attachment(1), first_attachment(2), first_attachment(3), 'Color', [colorCode_black, transparencyRatio*1.0], 'Marker', 's', 'LineWidth', 2);
        end
        

        % plot thruster
        % step 1: represent the thruster base locations and tip locations in the base frame
        base_loc_0 = zeros(size(mdl.propeller_data_locations));
        tip_loc_0 = zeros(size(mdl.propeller_data_locations));
        num_thrusters = size(mdl.propeller_data_locations,2);
        for k = 0:mdl.numLinks
            if k == 0
                R = eye(3);
                r_OP0 = [0; 0; 0];
            else
                R = QuaternionToRotationMatrix(mdl.quat_links(:,k))';
                r_OP0 = R*mdl.OP_links(:,k);
            end
            for i = 1:num_thrusters
                t_index = mdl.propellers(i);
                if mdl.propeller_data_link_indices(t_index) == k
                    base_loc_0(:,t_index) = r_OP0 + R*mdl.propeller_data_locations(:,t_index);
                    tip_loc_0(:,t_index) = base_loc_0(:,t_index) - 0.1*R*mdl.propeller_data_directions(:,t_index);
                end
            end
        end
        %step 2: plot each thruster
        for k = 1:num_thrusters
            t_index = mdl.propellers(k);
            base_k = base_loc_0(:,t_index);
            tip_k = tip_loc_0(:,t_index);
            plot3(ax, [base_k(1) tip_k(1)], [base_k(2) tip_k(2)], [base_k(3) tip_k(3)], 'Color', [colorCode_green, transparencyRatio*1.0], 'LineWidth', 1);
            plot3(ax, tip_k(1), tip_k(2), tip_k(3), 'Color', [colorCode_magenta, transparencyRatio*1.0], 'Marker', '*', 'LineWidth', 2);
        end

        view(view_angle);
        axis(plot_axis);
        pbaspect([plot_axis(2)-plot_axis(1) plot_axis(4)-plot_axis(3) plot_axis(6)-plot_axis(5)]);
        
        hold off;
    end
    

    % function that plots the robot frame. plot_axis represents the axis limits and view_angle represents the view angle as the name suggests
    %   This function is converted from MotionSimulator.PlotFrame() in CASPR
    function PlotFrameOp(mdl, op, plot_axis, view_angle, fig_handle, axis_handle, transparencyRatio)
        % color code: 
        colorCode_black = [0, 0, 0];
        colorCode_red = [1, 0, 0];
        colorCode_green = [0, 1, 0];
        colorCode_blue = [0, 0, 1];
        colorCode_magenta = [1, 0, 1];
        colorCode_cyan = [0, 1, 1];
        
        if nargin < 5 || isempty(fig_handle)
            figure;
        else
            figure(fig_handle);
        end
        if nargin < 6 || isempty(axis_handle)
            clf;
            ax = gca;
        else
            ax = axis_handle;
            axes(ax);
        end
        if nargin < 6
            transparencyRatio = 1.0;
        end
        
        hold on;
        grid on;
        xlabel('x (m)');
        ylabel('y (m)');
        zlabel('z (m)');
        
        % plot link
        for k = 1:mdl.numLinks
            R = QuaternionToRotationMatrix(mdl.quat_links(:,k))';
            r_OP0 = R*mdl.OP_links(:,k);
            r_OG0 = r_OP0 + R*mdl.CoM(:,k);
            body = [];
            VLk = mdl.VL{k};
            for i = 1:length(VLk)
                body = [body, r_OP0 + R*Polyhedron(VLk{i})];
            end
            plot3(ax,r_OP0(1), r_OP0(2), r_OP0(3), 'Color', [colorCode_black, transparencyRatio*1.0], 'Marker', 'o', 'LineWidth', 2);
            plot3(ax,r_OG0(1), r_OG0(2), r_OG0(3), 'Color', [colorCode_blue, transparencyRatio*1.0], 'Marker', 'o', 'LineWidth', 2);
            for i = 1:length(body)
                if body(i).isFullDim
                    body(i).plot('LineColor', 'b', 'LineWidth', 0.5, 'Color', 'k', 'alpha', transparencyRatio*0.3, 'edgealpha',transparencyRatio*0.8);
                else
                    body(i).plot('LineColor', 'k', 'LineWidth', 3, 'Color', 'k', 'alpha', transparencyRatio*0.1);
                end
            end
        end

        % plot cable
        % step 1: represent all the attachment locations in the base frame
        attachment_loc_0 = zeros(size(mdl.cable_data_attachments));
        num_attachment = size(mdl.cable_data_attachments,2);
        for k = 0:mdl.numLinks
            if k == 0
                R = eye(3);
                r_OP0 = [0; 0; 0];
            else
                R = QuaternionToRotationMatrix(mdl.quat_links(:,k))';
                r_OP0 = R*mdl.OP_links(:,k);
            end
            for i = 1:num_attachment
                if mdl.cable_data_attachment_link_indices(i) == k
                    attachment_loc_0(:,i) = r_OP0 + R*mdl.cable_data_attachments(:,i);
                end
            end
        end
        %step 2: plot each cable
        for k = 1:mdl.numCables
            attachment_set_k = mdl.cables{k};
            prev_attachment = attachment_loc_0(:,attachment_set_k(1));
            for i = 2:length(attachment_set_k)
                current_attachment = attachment_loc_0(:,attachment_set_k(i));
                plot3(ax, [prev_attachment(1) current_attachment(1)], [prev_attachment(2) current_attachment(2)], [prev_attachment(3) current_attachment(3)], 'Color', [colorCode_red, transparencyRatio*1.0], 'LineWidth', 1);
                prev_attachment = current_attachment;
            end
            first_attachment = attachment_loc_0(:,attachment_set_k(1));
            plot3(ax, first_attachment(1), first_attachment(2), first_attachment(3), 'Color', [colorCode_black, transparencyRatio*1.0], 'Marker', 's', 'LineWidth', 2);
        end
        

        % plot thruster
        % step 1: represent the thruster base locations and tip locations in the base frame
        base_loc_0 = zeros(size(mdl.propeller_data_locations));
        tip_loc_0 = zeros(size(mdl.propeller_data_locations));
        num_thrusters = size(mdl.propeller_data_locations,2);
        for k = 0:mdl.numLinks
            if k == 0
                R = eye(3);
                r_OP0 = [0; 0; 0];
            else
                R = QuaternionToRotationMatrix(mdl.quat_links(:,k))';
                r_OP0 = R*mdl.OP_links(:,k);
            end
            for i = 1:num_thrusters
                t_index = mdl.propellers(i);
                if mdl.propeller_data_link_indices(t_index) == k
                    base_loc_0(:,t_index) = r_OP0 + R*mdl.propeller_data_locations(:,t_index);
                    tip_loc_0(:,t_index) = base_loc_0(:,t_index) - 0.1*R*mdl.propeller_data_directions(:,t_index);
                end
            end
        end
        %step 2: plot each thruster
        for k = 1:num_thrusters
            t_index = mdl.propellers(k);
            base_k = base_loc_0(:,t_index);
            tip_k = tip_loc_0(:,t_index);
            plot3(ax, [base_k(1) tip_k(1)], [base_k(2) tip_k(2)], [base_k(3) tip_k(3)], 'Color', [colorCode_green, transparencyRatio*1.0], 'LineWidth', 1);
            plot3(ax, tip_k(1), tip_k(2), tip_k(3), 'Color', [colorCode_magenta, transparencyRatio*1.0], 'Marker', '*', 'LineWidth', 2);
        end
        
        % plot operational space
        for k = 1:op.numSegment
            if op.segmentType(k) == 2 % position
                y_tmp = op.mdl.pointCoordinate(op.linkIndices(k), op.referencePoints(:,k), 0);
                plot3(ax, y_tmp(1), y_tmp(2), y_tmp(3), 'Color', [colorCode_cyan, transparencyRatio*1.0], 'Marker', 'o', 'LineWidth', 2);
            end
        end

        view(view_angle);
        axis(plot_axis);
        pbaspect([plot_axis(2)-plot_axis(1) plot_axis(4)-plot_axis(3) plot_axis(6)-plot_axis(5)]);
        
        hold off;
    end


    function PlotMovie(mdl, q_series, time_series, plot_axis, view_angle, folder_name, filename, traj_visualization_set, q_ref_series, time, width, height)
        trajectory_duration = time_series(end);
        trajectory_length = length(time_series);
        if (nargin < 8) || isempty(traj_visualization_set)
            flag_traj_visualization = false;
        else
            flag_traj_visualization = true;
            numPoints = length(traj_visualization_set);
            bg = {'k', 'b', 'g', 'r', 'w', 'c', [1 0 1]};
            colors = distinguishable_colors(numPoints,bg);
            pt_traj = cell(numPoints, 1);
        end
        if (nargin < 9) || isempty(traj_visualization_set) || isempty(q_ref_series)
            flag_ref_visualization = false;
        else
            flag_ref_visualization = true;
            pt_ref_traj = cell(numPoints, 1);
        end
        if (nargin < 10)
            % Default time to be time of trajectory (no speed up or
            % slow down)
            time = trajectory_duration;
        end
        if (nargin < 11)
            width = ModelVisualization.MOVIE_DEFAULT_WIDTH;
            height = ModelVisualization.MOVIE_DEFAULT_HEIGHT;
        end
        fps = 30;

        % fill in the complete file path to the video file
        [current_folder,~,~] = fileparts(mfilename('fullpath'));
        path_to_file_folder       	=   [current_folder, '\..\..\local_files\videos', '\', mdl.robotName,folder_name];
        if ~exist(path_to_file_folder, 'dir')
            mkdir(path_to_file_folder)
        end
        complete_file_path    =   [path_to_file_folder, '\', filename];
        
        writerObj = VideoWriter(complete_file_path);
        % writerObj.Quality = 100;
        writerObj.open();
        plot_handle = figure('Position', [10, 10, width, height]);
        zero_vec = zeros(mdl.numDofs,1);
        for i = 1:round(fps*time)
            t = round(trajectory_length/round(fps*time)*i);
            if t == 0
                t = 1;
            end
            mdl.update(q_series{t}, zero_vec, zero_vec, zero_vec);
            if flag_traj_visualization
                for j = 1:numPoints
                    link = traj_visualization_set(j).pt_link;
                    pt_coordinate = traj_visualization_set(j).pt_coordinate;
                    pt_traj{j} = [pt_traj{j}; mdl.pointCoordinate(link, pt_coordinate, 0)'];
                end
            end
            ModelVisualization.PlotFrame(mdl, plot_axis, view_angle, plot_handle);
            if flag_ref_visualization
                mdl.update(q_ref_series{t}, zero_vec, zero_vec, zero_vec);
                for j = 1:numPoints
                    link = traj_visualization_set(j).pt_link;
                    pt_coordinate = traj_visualization_set(j).pt_coordinate;
                    pt_ref_traj{j} = [pt_ref_traj{j}; mdl.pointCoordinate(link, pt_coordinate, 0)'];
                end
            end
            hold on;
            figure(plot_handle);
            ax = gca;
            if flag_ref_visualization
                for j = 1:numPoints
                    plot3(ax, pt_ref_traj{j}(:,1), pt_ref_traj{j}(:,2), pt_ref_traj{j}(:,3), 'Color', 0.7*colors(j,:), 'LineWidth', 2, 'LineStyle', '--');
                end
            end
            if flag_traj_visualization
                for j = 1:numPoints
                    plot3(ax, pt_traj{j}(:,1), pt_traj{j}(:,2), pt_traj{j}(:,3), 'Color', colors(j,:), 'LineWidth', 1, 'LineStyle', '-');
                end
            end
            frame = getframe(plot_handle);
            writerObj.writeVideo(frame);
        end
        writerObj.close();
        close(plot_handle);
    end
    
    
    function PlotMovieReconfigurable(mdl, reconfigSys, q_series, config_param_series, time_series, plot_axis, view_angle, folder_name, filename, traj_visualization_set, q_ref_series, time, width, height)
        trajectory_duration = time_series(end);
        trajectory_length = length(time_series);
        if (nargin < 10) || isempty(traj_visualization_set)
            flag_traj_visualization = false;
        else
            flag_traj_visualization = true;
            numPoints = length(traj_visualization_set);
            bg = {'k', 'b', 'g', 'r', 'w', 'c', [1 0 1]};
            colors = distinguishable_colors(numPoints,bg);
            pt_traj = cell(numPoints, 1);
        end
        if (nargin < 11) || isempty(traj_visualization_set) || isempty(q_ref_series)
            flag_ref_visualization = false;
        else
            flag_ref_visualization = true;
            pt_ref_traj = cell(numPoints, 1);
        end
        if (nargin < 12)
            % Default time to be time of trajectory (no speed up or
            % slow down)
            time = trajectory_duration;
        end
        if (nargin < 13)
            width = ModelVisualization.MOVIE_DEFAULT_WIDTH;
            height = ModelVisualization.MOVIE_DEFAULT_HEIGHT;
        end
        fps = 30;

        % fill in the complete file path to the video file
        [current_folder,~,~] = fileparts(mfilename('fullpath'));
        path_to_file_folder       	=   [current_folder, '\..\..\local_files\videos', '\', mdl.robotName,folder_name];
        if ~exist(path_to_file_folder, 'dir')
            mkdir(path_to_file_folder)
        end
        complete_file_path    =   [path_to_file_folder, '\', filename];
        
        writerObj = VideoWriter(complete_file_path);
        %writerObj.Quality = 100;
        writerObj.open();
        plot_handle = figure('Position', [10, 10, width, height]);
        zero_vec = zeros(mdl.numDofs,1);
        for i = 1:round(fps*time)
            t = round(trajectory_length/round(fps*time)*i);
            if t == 0
                t = 1;
            end
            mdl.update(q_series{t}, zero_vec, zero_vec, zero_vec);
            if flag_traj_visualization
                for j = 1:numPoints
                    link = traj_visualization_set(j).pt_link;
                    pt_coordinate = traj_visualization_set(j).pt_coordinate;
                    pt_traj{j} = [pt_traj{j}; mdl.pointCoordinate(link, pt_coordinate, 0)'];
                end
            end
            reconfigSys.applyConfiguration(mdl, config_param_series{t});
            ModelVisualization.PlotFrame(mdl, plot_axis, view_angle, plot_handle);
            mdl.update(q_ref_series{t}, zero_vec, zero_vec, zero_vec);
            if flag_ref_visualization
                for j = 1:numPoints
                    link = traj_visualization_set(j).pt_link;
                    pt_coordinate = traj_visualization_set(j).pt_coordinate;
                    pt_ref_traj{j} = [pt_ref_traj{j}; mdl.pointCoordinate(link, pt_coordinate, 0)'];
                end
            end
            hold on;
            figure(plot_handle);
            ax = gca;
            if flag_ref_visualization
                for j = 1:numPoints
                    plot3(ax, pt_ref_traj{j}(:,1), pt_ref_traj{j}(:,2), pt_ref_traj{j}(:,3), 'Color', 0.7*colors(j,:), 'LineWidth', 2, 'LineStyle', '--');
                end
            end
            if flag_traj_visualization
                for j = 1:numPoints
                    plot3(ax, pt_traj{j}(:,1), pt_traj{j}(:,2), pt_traj{j}(:,3), 'Color', colors(j,:), 'LineWidth', 1, 'LineStyle', '-');
                end
            end
            frame = getframe(plot_handle);
            writerObj.writeVideo(frame);
        end
        writerObj.close();
        close(plot_handle);
    end

    


    function PlotMovieOp(mdl, op, q_series, time_series, plot_axis, view_angle, folder_name, filename, traj_visualization_set, y_ref_series, time, width, height)
        trajectory_duration = time_series(end);
        trajectory_length = length(time_series);
        if (nargin < 9) || isempty(traj_visualization_set)
            flag_traj_visualization = false;
        else
            flag_traj_visualization = true;
            numPoints = length(traj_visualization_set);
            bg = {'k', 'b', 'g', 'r', 'w', 'c', [1 0 1]};
            colors = distinguishable_colors(numPoints,bg);
            pt_traj = cell(numPoints, 1);
        end
        if (nargin < 10) || isempty(traj_visualization_set) || isempty(y_ref_series)
            flag_ref_visualization = false;
        else
            flag_ref_visualization = true;
            pt_ref_traj = cell(numPoints, 1);
        end
        if (nargin < 11)
            % Default time to be time of trajectory (no speed up or
            % slow down)
            time = trajectory_duration;
        end
        if (nargin < 12)
            width = ModelVisualization.MOVIE_DEFAULT_WIDTH;
            height = ModelVisualization.MOVIE_DEFAULT_HEIGHT;
        end
        fps = 30;

        % fill in the complete file path to the video file
        [current_folder,~,~] = fileparts(mfilename('fullpath'));
        path_to_file_folder       	=   [current_folder, '\..\..\local_files\videos', '\', mdl.robotName, folder_name];
        if ~exist(path_to_file_folder, 'dir')
            mkdir(path_to_file_folder)
        end
        complete_file_path    =   [path_to_file_folder, '\', filename];
        
        writerObj = VideoWriter(complete_file_path);
        %writerObj.Quality = 100;
        writerObj.open();
        plot_handle = figure('Position', [10, 10, width, height]);
        zero_vec = zeros(mdl.numDofs,1);

        for i = 1:round(fps*time)
            t = round(trajectory_length/round(fps*time)*i);
            if t == 0
                t = 1;
            end
            mdl.update(q_series{t}, zero_vec, zero_vec, zero_vec);
            ModelVisualization.PlotFrameOp(mdl, op, plot_axis, view_angle, plot_handle);
            hold on;
            figure(plot_handle);
            ax = gca;
            if flag_traj_visualization
                for j = 1:numPoints
                    link = traj_visualization_set(j).pt_link;
                    pt_coordinate = traj_visualization_set(j).pt_coordinate;
                    pt_traj{j} = [pt_traj{j}; mdl.pointCoordinate(link, pt_coordinate, 0)'];
                    plot3(ax, pt_traj{j}(:,1), pt_traj{j}(:,2), pt_traj{j}(:,3), 'Color', colors(j,:), 'LineWidth', 1.0, 'LineStyle', '-');
                end
            end
            if flag_ref_visualization
                for j = 1:numPoints
                    pt = pt_traj{j}(end,:);
                    pt(traj_visualization_set(j).cartesian_index) = y_ref_series{t}(traj_visualization_set(j).op_index)';
                    pt_ref_traj{j} = [pt_ref_traj{j}; pt];
                    plot3(ax, pt_ref_traj{j}(:,1), pt_ref_traj{j}(:,2), pt_ref_traj{j}(:,3), 'Color', 0.7*colors(j,:), 'LineWidth', 2, 'LineStyle', '--');
                end
            end
            frame = getframe(plot_handle);
            writerObj.writeVideo(frame);
        end
        writerObj.close();
        close(plot_handle);
    end
end
    
end