% Input: Weightage matrix
weightage_matrix = ceil(avg_height_grid / 0.05);
[rows, cols] = size(weightage_matrix);

% Initialize
start_position = [1, 1]; % Bottom-left corner
current_position = start_position;
distance_traveled = 0;
path = current_position; % Store the path

% Define movement directions (up, down, left, right)
directions = [-1, 0; 1, 0; 0, -1; 0, 1];

% While there are grids left to visit
while any(weightage_matrix(:) > 0)
    max_height = -1;
    next_position = [];
    
    % Check all 4 directions
    for d = 1:4
        neighbor = current_position + directions(d, :);
        
        % Ensure the neighbor is within bounds
        if neighbor(1) >= 1 && neighbor(1) <= rows && neighbor(2) >= 1 && neighbor(2) <= cols
            neighbor_height = weightage_matrix(neighbor(1), neighbor(2));
            
            % Prioritize the grid with the highest remaining weightage
            if neighbor_height > max_height
                max_height = neighbor_height;
                next_position = neighbor;
            end
        end
    end
    
    % Move to the next position
    if ~isempty(next_position)
        % Update path and distance traveled
        path = [path; next_position];
        distance_traveled = distance_traveled + norm(next_position - current_position);
        current_position = next_position;
        
        % Reduce the weightage of the visited grid
        weightage_matrix(current_position(1), current_position(2)) = ...
            weightage_matrix(current_position(1), current_position(2)) - 1;
    else
        error('No valid moves found, check the algorithm logic!');
    end
end

% Output results
fprintf('Total Distance Traveled: %.2f meters\n', distance_traveled);

% Visualization
figure;
imagesc(weightage_matrix); hold on;
plot(path(:, 2), path(:, 1), 'r-o', 'LineWidth', 2);
title('Rover Path');
xlabel('X (meters)');
ylabel('Y (meters)');