function [axis_positions, axis_positions_absolute] = TighterSubplotAxisDefinition(numRow, numCol, left_label_margin, bottom_label_margin, left_axis_margin, bottom_axis_margin, right_normal_margin, top_normal_margin)

    % bottom_label_margin, bottom_axis_margin and top_normal_margin are
    % vectors of length numRow
    % left_label_margin, left_axis_margin and right_normal_margin are
    % vectors of length numCol

    % bottom_label_margin and bottom_axis_margin should be arranged top to
    % bottom row-wise
    
    axis_positions = cell(numRow, numCol);
    axis_positions_absolute = cell(numRow, numCol);
    
    box_width = (1-sum(right_normal_margin)-sum(left_label_margin)-sum(left_axis_margin))/numCol;
    box_height = (1-sum(top_normal_margin)-sum(bottom_label_margin)-sum(bottom_axis_margin))/numRow;
    
    for i = 1:numRow
        for j = 1:numCol
            if i == 1
                offset_vertical = 0;
            else
                offset_vertical = axis_positions_absolute{i-1,j}(4)+top_normal_margin(numRow-(i-1)+1);
            end
            if j == 1
                offset_horizontal = 0;
            else
                offset_horizontal = axis_positions_absolute{i,j-1}(3)+right_normal_margin(j-1);
            end
            axis_positions_absolute{i,j}(1) = offset_horizontal + left_label_margin(j) + left_axis_margin(j);
            axis_positions_absolute{i,j}(2) = offset_vertical + bottom_label_margin(numRow-i+1) + bottom_axis_margin(numRow-i+1);
            axis_positions_absolute{i,j}(3) = axis_positions_absolute{i,j}(1) + box_width;
            axis_positions_absolute{i,j}(4) = axis_positions_absolute{i,j}(2) + box_height;
        end
    end
    
    tmp = axis_positions_absolute;
    for i = 1:numRow
        for j = 1:numCol
            axis_positions_absolute{i,j} = tmp{numRow-i+1, j};
            axis_positions{i,j} = tmp{numRow-i+1, j};
            axis_positions{i,j}(3) = box_width;
            axis_positions{i,j}(4) = box_height;
        end
    end

end