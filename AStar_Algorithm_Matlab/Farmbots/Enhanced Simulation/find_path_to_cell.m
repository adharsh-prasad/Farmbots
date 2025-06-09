function path = find_path_to_cell(height_grid, start_pos, end_pos, visit_count)
    [rows, cols] = size(height_grid);
    
    % Create a grid for pathfinding
    cost_grid = height_grid + visit_count;
    
    % Initialize the open and closed lists
    open_list = PriorityQueue();
    closed_list = zeros(rows, cols);
    
    % Add the starting node to the open list
    open_list.push(start_pos, 0);
    
    % Initialize the parent map
    parent_map = containers.Map('KeyType', 'char', 'ValueType', 'any');
    
    while ~open_list.isEmpty()
        current = open_list.pop();
        
        if isequal(current, end_pos)
            % Path found, reconstruct and return it
            path = reconstruct_path(parent_map, start_pos, end_pos);
            return;
        end
        
        closed_list(current(1), current(2)) = 1;
        
        % Check neighbors
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0
                    continue;
                end
                
                neighbor = [current(1) + dx, current(2) + dy];
                
                if neighbor(1) < 1 || neighbor(1) > rows || neighbor(2) < 1 || neighbor(2) > cols
                    continue;
                end
                
                if closed_list(neighbor(1), neighbor(2))
                    continue;
                end
                
                cost = cost_grid(neighbor(1), neighbor(2));
                open_list.push(neighbor, cost);
                
                key = sprintf('%d,%d', neighbor(1), neighbor(2));
                parent_map(key) = current;
            end
        end
    end
    
    % No path found
    path = [];
end

function path = reconstruct_path(parent_map, start_pos, end_pos)
    path = [end_pos];
    current = end_pos;
    
    while ~isequal(current, start_pos)
        key = sprintf('%d,%d', current(1), current(2));
        current = parent_map(key);
        path = [current; path];
    end
end