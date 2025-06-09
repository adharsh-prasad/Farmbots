% Calculate initial path
[path, visit_count] = terrain_leveling_astar(avg_height_grid, [1, 1]);

% Visualize the path and terrain
figure(1);
imagesc(avg_height_grid);
hold on;
plot(path(:,2), path(:,1), 'r-', 'LineWidth', 2);
plot(path(1,2), path(1,1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(path(end,2), path(end,1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
colorbar;
title('Optimal Leveling Path');
xlabel('X Grid');
ylabel('Y Grid');

figure(2);
imagesc(visit_count);
colorbar;
title('Visit Count Map');
xlabel('X Grid');
ylabel('Y Grid');