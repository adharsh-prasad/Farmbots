function [path, visit_count] = terrain_leveling_astar(height_grid, start_pos)
    % Initialize grid dimensions
    [rows, cols] = size(height_grid);
    
    % Initialize visit tracking and required passes matrices
    visit_count = zeros(rows, cols);
    required_passes = ceil((height_grid - min(height_grid(:))) / 0.05);
    
    % Initialize path and current position
    path = [];
    current_pos = start_pos;
    unvisited_cells = true;
    
    while unvisited_cells
        % Find next highest priority cell
        [next_pos, priority] = find_next_priority_cell(height_grid, visit_count, required_passes, current_pos);
        
        if isempty(next_pos)
            break;
        end
        
        % Generate path to next position
        segment_path = find_path_to_cell(height_grid, current_pos, next_pos, visit_count);
        
        if ~isempty(segment_path)
            % Add segment to full path
            path = [path; segment_path];
            
            % Update visit count for cells in path
            for i = 1:size(segment_path, 1)
                cell_pos = segment_path(i,:);
                visit_count(cell_pos(1), cell_pos(2)) = visit_count(cell_pos(1), cell_pos(2)) + 1;
            end
            
            % Update current position
            current_pos = next_pos;
        end
        
        % Check if all cells have been visited enough times
        unvisited_cells = ~all(visit_count >= required_passes, 'all');
    end
end

function [next_pos, priority] = find_next_priority_cell(height_grid, visit_count, required_passes, current_pos)
    [rows, cols] = size(height_grid);
    max_priority = -inf;
    next_pos = [];
    
    for i = 1:rows
        for j = 1:cols
            if visit_count(i,j) < required_passes(i,j)
                % Calculate priority based on height and distance
                height_priority = height_grid(i,j);
                distance = norm([i,j] - current_pos);
                coverage_need = required_passes(i,j) - visit_count(i,j);
                
                % Priority formula: higher elevation and fewer visits increase priority
                priority = (height_priority * coverage_need) - (0.1 * distance);
                
                if priority > max_priority
                    max_priority = priority;
                    next_pos = [i,j];
                end
            end
        end
    end
end

function path = find_path_to_cell(height_grid, start_pos, goal_pos, visit_count)
    [rows, cols] = size(height_grid);
    
    % Initialize costs
    g_score = inf(rows, cols);
    f_score = inf(rows, cols);
    g_score(start_pos(1), start_pos(2)) = 0;
    f_score(start_pos(1), start_pos(2)) = manhattan_distance(start_pos, goal_pos);
    
    % Initialize open and closed sets
    open_set = {start_pos};
    closed_set = {};
    came_from = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    while ~isempty(open_set)
        current = get_lowest_f_score(open_set, f_score);
        
        if isequal(current, goal_pos)
            path = reconstruct_path(came_from, current);
            return;
        end
        
        open_set = remove_from_set(open_set, current);
        closed_set{end+1} = current;
        
        % Check neighbors (8-connected)
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0
                    continue;
                end
                
                neighbor = current + [dx, dy];
                
                if ~is_valid_position(neighbor, rows, cols) || is_in_set(closed_set, neighbor)
                    continue;
                end
                
                tentative_g = g_score(current(1), current(2)) + ...
                    calculate_cost(height_grid, current, neighbor, visit_count);
                
                if ~is_in_set(open_set, neighbor)
                    open_set{end+1} = neighbor;
                elseif tentative_g >= g_score(neighbor(1), neighbor(2))
                    continue;
                end
                
                % Record this path
                came_from(pos2key(neighbor)) = current;
                g_score(neighbor(1), neighbor(2)) = tentative_g;
                f_score(neighbor(1), neighbor(2)) = tentative_g + ...
                    manhattan_distance(neighbor, goal_pos);
            end
        end
    end
    path = [];
end

function current = get_lowest_f_score(open_set, f_score)
    lowest_f = inf;
    current = [];
    
    for i = 1:length(open_set)
        pos = open_set{i};
        if f_score(pos(1), pos(2)) < lowest_f
            lowest_f = f_score(pos(1), pos(2));
            current = pos;
        end
    end
end

function d = manhattan_distance(pos1, pos2)
    d = abs(pos1(1) - pos2(1)) + abs(pos1(2) - pos2(2));
end

function cost = calculate_cost(height_grid, current, neighbor, visit_count)
    base_cost = 1;
    height_diff = abs(height_grid(neighbor(1), neighbor(2)) - height_grid(current(1), current(2)));
    visit_penalty = visit_count(neighbor(1), neighbor(2)) * 0.5;
    
    cost = base_cost + (height_diff * 10) + visit_penalty;
end

function valid = is_valid_position(pos, rows, cols)
    valid = pos(1) >= 1 && pos(1) <= rows && pos(2) >= 1 && pos(2) <= cols;
end

function in_set = is_in_set(set, pos)
    in_set = false;
    for i = 1:length(set)
        if isequal(set{i}, pos)
            in_set = true;
            return;
        end
    end
end

function new_set = remove_from_set(set, pos)
    new_set = {};
    for i = 1:length(set)
        if ~isequal(set{i}, pos)
            new_set{end+1} = set{i};
        end
    end
end

function key = pos2key(pos)
    key = sprintf('%d,%d', pos(1), pos(2));
end

function path = reconstruct_path(came_from, current)
    path = current;
    current_key = pos2key(current);
    
    while isKey(came_from, current_key)
        current = came_from(current_key);
        path = [current; path];
        current_key = pos2key(current);
    end
end