function [path, visit_count] = Updated_terrain_leveling_astar(height_grid, start_pos)
    % Initialize grid dimensions
    [rows, cols] = size(height_grid);
    
    % Calculate target height (mean height)
    target_height = 0;
    
    % Calculate required passes for each cell based on height difference
    leveling_per_pass = 0.05; % Height reduction per pass
    required_passes = ceil((height_grid - target_height) / leveling_per_pass);
    required_passes = max(required_passes, 1); % Minimum one pass
    
    % Initialize tracking matrices
    visit_count = zeros(rows, cols);
    path = [];
    current_pos = start_pos;
    
    % Continue until all cells have required passes
    while any(visit_count < required_passes, 'all')
        % Find next priority cell
        [next_pos, priority] = find_next_priority_cell(height_grid, visit_count, required_passes, current_pos);
        
        if isempty(next_pos)
            break;
        end
        
        % Generate path to next position
        segment_path = find_path_to_cell(height_grid, current_pos, next_pos, visit_count, required_passes);
        
        if ~isempty(segment_path)
            % Add segment to full path
            path = [path; segment_path];
            
            % Update visit counts along path
            for i = 1:size(segment_path, 1)
                cell_pos = segment_path(i,:);
                visit_count(cell_pos(1), cell_pos(2)) = visit_count(cell_pos(1), cell_pos(2)) + 1;
            end
            
            current_pos = next_pos;
        end
    end
end

function [next_pos, priority] = find_next_priority_cell(height_grid, visit_count, required_passes, current_pos)
    [rows, cols] = size(height_grid);
    max_priority = -inf;
    next_pos = [];
    
    for i = 1:rows
        for j = 1:cols
            if visit_count(i,j) < required_passes(i,j)
                % Calculate priority based on height, remaining passes needed, and distance
                height_priority = height_grid(i,j);
                distance = norm([i,j] - current_pos);
                remaining_passes = required_passes(i,j) - visit_count(i,j);
                
                % Priority formula favors higher elevation and more remaining passes
                priority = (height_priority * remaining_passes) - (0.1 * distance);
                
                if priority > max_priority
                    max_priority = priority;
                    next_pos = [i,j];
                end
            end
        end
    end
end

function path = find_path_to_cell(height_grid, start_pos, goal_pos, visit_count, required_passes)
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
        
        % Check all 8 neighbors
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
                    calculate_cost(height_grid, visit_count, required_passes, current, neighbor);
                
                if ~is_in_set(open_set, neighbor)
                    open_set{end+1} = neighbor;
                elseif tentative_g >= g_score(neighbor(1), neighbor(2))
                    continue;
                end
                
                came_from(pos2key(neighbor)) = current;
                g_score(neighbor(1), neighbor(2)) = tentative_g;
                f_score(neighbor(1), neighbor(2)) = tentative_g + manhattan_distance(neighbor, goal_pos);
            end
        end
    end
    path = [];
end

function d = manhattan_distance(pos1, pos2)
    % Calculate Manhattan distance between two positions
    d = abs(pos1(1) - pos2(1)) + abs(pos1(2) - pos2(2));
end

function current = get_lowest_f_score(open_set, f_score)
    % Initialize variables
    lowest_f = inf;
    current = [];
    
    % Search through open set for node with lowest f_score
    for i = 1:length(open_set)
        pos = open_set{i};
        if f_score(pos(1), pos(2)) < lowest_f
            lowest_f = f_score(pos(1), pos(2));
            current = pos;
        end
    end
end

function path = reconstruct_path(came_from, current)
    % Initialize path with current node
    path = current;
    current_key = pos2key(current);
    
    % Trace back the path from goal to start
    while isKey(came_from, current_key)
        current = came_from(current_key);
        path = [current; path];
        current_key = pos2key(current);
    end
end

function key = pos2key(pos)
    key = sprintf('%d,%d', pos(1), pos(2));
end

function new_set = remove_from_set(set, pos)
    new_set = {};
    for i = 1:length(set)
        if ~isequal(set{i}, pos)
            new_set{end+1} = set{i};
        end
    end
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

function cost = calculate_cost(height_grid, visit_count, required_passes, current, neighbor)
    % Base movement cost
    base_cost = 1;
    
    % Height difference cost
    current_height = height_grid(current(1), current(2));
    neighbor_height = height_grid(neighbor(1), neighbor(2));
    height_diff = abs(neighbor_height - current_height);
    height_cost = height_diff * 10;
    
    % Visit penalty - higher cost for cells that have been visited more times
    current_visits = visit_count(neighbor(1), neighbor(2));
    required_visits = required_passes(neighbor(1), neighbor(2));
    remaining_visits = max(0, required_visits - current_visits);
    visit_penalty = current_visits / (remaining_visits + 0.1);
    
    % Combined cost
    cost = base_cost + height_cost + visit_penalty;
end