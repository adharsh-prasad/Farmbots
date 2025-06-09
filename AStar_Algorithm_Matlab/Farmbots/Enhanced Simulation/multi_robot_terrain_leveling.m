function [paths, visit_count] = multi_robot_terrain_leveling(height_grid, start_positions, num_robots)
    [rows, cols] = size(height_grid);
    visit_count = zeros(rows, cols);
    required_passes = ceil((height_grid - min(height_grid(:))) / 0.05);
    paths = cell(1, num_robots);
    current_positions = start_positions;
    
    max_iterations = rows * cols * max(required_passes(:)); % Set a maximum number of iterations
    
    for iteration = 1:max_iterations
        unvisited_cells = find(visit_count < required_passes);
        if isempty(unvisited_cells)
            break;
        end
        
        for robot = 1:num_robots
            [next_pos, priority] = find_next_priority_cell(height_grid, visit_count, required_passes, current_positions(robot,:), current_positions(setdiff(1:num_robots, robot),:));
            
            if ~isempty(next_pos)
                segment_path = find_path_to_cell(height_grid, current_positions(robot,:), next_pos, visit_count);
                
                if ~isempty(segment_path)
                    paths{robot} = [paths{robot}; segment_path];
                    
                    for i = 1:size(segment_path, 1)
                        cell_pos = segment_path(i,:);
                        visit_count(cell_pos(1), cell_pos(2)) = visit_count(cell_pos(1), cell_pos(2)) + 1;
                    end
                    
                    current_positions(robot,:) = next_pos;
                end
            end
        end
        
        % Check if all robots are stuck (no valid next positions)
        if all(cellfun(@isempty, paths))
            break;
        end
    end
    
    if iteration == max_iterations
        warning('Maximum iterations reached. The terrain may not be fully leveled.');
    end
end


function [next_pos, priority] = find_next_priority_cell(height_grid, visit_count, required_passes, current_pos, other_robot_positions)

    [rows, cols] = size(height_grid);
    max_priority = -inf;
    next_pos = [];
    
    for i = 1:rows
        for j = 1:cols
            if visit_count(i,j) < required_passes(i,j)
                height_priority = height_grid(i,j);
                distance = norm([i,j] - current_pos);
                coverage_need = required_passes(i,j) - visit_count(i,j);
                
                % Check distance to other robots
                min_robot_distance = min(pdist2([i,j], other_robot_positions));
                robot_avoidance_factor = 1 / (1 + exp(-min_robot_distance + 5)); % Sigmoid function
                
                priority = (height_priority * coverage_need * robot_avoidance_factor) - (0.1 * distance);
                
                if priority > max_priority
                    max_priority = priority;
                    next_pos = [i,j];
                end
            end
        end
    end
end