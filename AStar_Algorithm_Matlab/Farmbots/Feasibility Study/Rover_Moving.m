close all;

% Create a random map
mapSize = [50, 50];
map = rand(mapSize) > 0.8;  % 30% chance of obstacles
xlim([0, 50])
ylim([0, 50])
% Display the map
figure;
imagesc(map);
colormap([1 1 1; 0 0 0]);  % White for free space, black for obstacles
hold on;
axis equal;
title('Obstacle Map with Rover Path');

% Define start and goal positions
start = [1, 1];
goal = [mapSize(1), mapSize(2)];

% Plot start and goal
plot(start(2), start(1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Ensure start and goal are in free space
map(start(1), start(2)) = 0;
map(goal(1), goal(2)) = 0;

% Create distance transform
distanceMap = bwdistgeodesic(~map, goal(2), goal(1), 'quasi-euclidean');

% Find path
path = zeros(0, 2);
current = start;
while ~isequal(current, goal)
    path(end+1, :) = current;
    
    % Get 8-connected neighbors
    [row, col] = meshgrid(current(1)-1:current(1)+1, current(2)-1:current(2)+1);
    neighbors = [row(:), col(:)];
    
    % Remove out-of-bounds neighbors
    validIdx = all(neighbors > 0 & neighbors <= mapSize, 2);
    neighbors = neighbors(validIdx, :);
    
    % Find neighbor with minimum distance
    [~, idx] = min(distanceMap(sub2ind(size(map), neighbors(:,1), neighbors(:,2))));
    current = neighbors(idx, :);
end
path(end+1, :) = goal;

% Plot the path
plot(path(:,2), path(:,1), 'b-', 'LineWidth', 2);

% Simulate rover movement
rover = plot(start(2), start(1), 'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
for i = 1:size(path, 1)
    set(rover, 'XData', path(i,2), 'YData', path(i,1));
    drawnow;
    pause(0.1);
end

hold off;