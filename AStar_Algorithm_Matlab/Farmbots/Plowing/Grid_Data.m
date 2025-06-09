% Define grid parameters based on rover's plow radius
plow_radius = 0.2;
grid_size = 5;
num_cells = floor(grid_size / (2 * plow_radius));

% Initialize grid for average heights
avg_height_grid = zeros(num_cells, num_cells);
grid_count = zeros(num_cells, num_cells);

% Collect height data for each grid cell
for i = 1:num_cells
    for j = 1:num_cells
        % Calculate center coordinates of current grid cell
        x_center = (2 * plow_radius) * (j - 0.5);
        y_center = (2 * plow_radius) * (i - 0.5);
        
        % Find all points within this grid cell
        [local_X, local_Y] = meshgrid(x_center-plow_radius:0.01:x_center+plow_radius, ...
                                    y_center-plow_radius:0.01:y_center+plow_radius);
        mask = (local_X - x_center).^2 + (local_Y - y_center).^2 <= plow_radius^2;
        
        % Get heights for these points
        local_Z = interp2(X, Y, Z, local_X, local_Y);
        valid_heights = local_Z(mask);
        
        % Calculate average height for this cell
        if ~isempty(valid_heights)
            avg_height_grid(i, j) = mean(valid_heights, 'omitnan');
            grid_count(i, j) = sum(mask(:));
        end
    end
end

% Visualize the average height grid
figure;
imagesc(avg_height_grid);
colorbar;
title('Average Height Grid After Leveling');
xlabel('X Grid');
ylabel('Y Grid');
axis equal;
set(gca, 'YDir', 'normal');