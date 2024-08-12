function point_set = HyperrectangleVertices(lower_bound, upper_bound)

    dim = length(lower_bound);
    num_points = 2^dim;
    point_set = zeros(dim, num_points);
    offset = double('0');
    for i = 1:num_points
        % vertex_index = i;
        % for j = 1:dim
        %     % deal with the j-th element of the i-th vertex
        %     tmp = mod(vertex_index, 2);
        %     if tmp == 1
        %         point_set(j, i) = upper_bound(j);
        %     else
        %         point_set(j, i) = lower_bound(j);
        %     end
        %     vertex_index = floor(vertex_index/2);
        % end
        beta = double(dec2bin(i-1,dim))' - offset;
        point_set(:,i) = lower_bound.*(~beta) + upper_bound.*beta;
    end

end