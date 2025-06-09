% Clear workspace and close all figures 
close all; 

% Create a fullscreen figure without borders or menus
fig = figure('WindowState', 'fullscreen', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Name', '', ...
    'Color', 'k');

% Maximize the figure window to remove any gaps
set(fig, 'WindowStyle', 'normal');

% Create a high-resolution 5x5 meter grid
[X, Y] = meshgrid(0:0.01:10, 0:0.01:10);

% Generate the terrain using sinusoidal variations
amplitude_variation = 0.1 * rand(size(X));
frequency_variation_x = 2 * pi * (0.1 * rand(size(X)));
frequency_variation_y = 2 * pi * (0.1 * rand(size(Y)));
Z = amplitude_variation .* sin(frequency_variation_x .* X) .* cos(frequency_variation_y .* Y);

% Add bumps for texture
num_bumps = 2000;
for i = 1:num_bumps
    x_center = rand() * 10;
    y_center = rand() * 10;
    height = 1 * (0.005 + 0.025 * rand()) * randn();
    width = 0.1 + 0.2 * rand();
    Z = Z + height * exp(-((X - x_center).^2 + (Y - y_center).^2) / (2 * width^2));
end

% Ensure positive terrain and smooth with Gaussian filter
Z = abs(Z);
Z = imgaussfilt(Z, 1);

% Plot the terrain
surf_handle = surf(X, Y, Z, 'EdgeColor', 'none');
hold on;

% Set axis properties for fullscreen appearance
axis off;
view(2);

% Adjust limits to fit the entire grid
xlim([0 10]); 
ylim([0 10]);

% Set lighting and colormap
lighting gouraud
material dull
camlight('headlight');
shading interp;
colormap(parula);
light('Position', [1 1 1])
light('Position', [-1 -1 -1])


% Remove white space from the figure
set(gca, 'Position', [0 0 1 1]);


% Rover parameters
width = 0.3; height = 0.15; depth = 0.3;
rover_speed = 0.1;
plow_radius = 0.1;
grid_size = 10;

% Initialize two Rovers
initial_positions = [plow_radius, plow_radius, 0;
                     grid_size - plow_radius, plow_radius, 0];
for i = 1:2
    rover(i) = Rover(initial_positions(i,:), [width, height, depth], rover_speed, plow_radius);
end

% Generate row-wise path
path_x = [];
path_y = [];
for y = plow_radius:2*plow_radius:grid_size-plow_radius
    if mod(floor(y/(2*plow_radius)), 2) == 0
        x_range = plow_radius:2*plow_radius:grid_size-plow_radius;
    else
        x_range = grid_size-plow_radius:-2*plow_radius:plow_radius;
    end
    path_x = [path_x, x_range];
    path_y = [path_y, y * ones(1, length(x_range))];
end

% Split the path into two parts
half_length = floor(length(path_x)/2);
path_x = {path_x(1:half_length), path_x(half_length+1:end)};
path_y = {path_y(1:half_length), path_y(half_length+1:end)};

% Interpolate Z values for both paths and set the paths for the rovers
for i = 1:2
    path_z = interp2(X, Y, Z, path_x{i}, path_y{i});
    rover(i).setPath([path_x{i}', path_y{i}', path_z']);
end

tic
% Animate rovers along their paths and collect data
while any([rover.CurrentPathIndex] <= [size(rover(1).Path, 1), size(rover(2).Path, 1)])
    for i = 1:2
        if rover(i).CurrentPathIndex <= size(rover(i).Path, 1)
            x = rover(i).Position(1);
            y = rover(i).Position(2);
            rover(i).move();
            [local_X, local_Y] = meshgrid(x-plow_radius:0.01:x+plow_radius, y-plow_radius:0.01:y+plow_radius);
            mask = (local_X - x).^2 + (local_Y - y).^2 <= plow_radius^2;
            local_Z = interp2(X, Y, Z, local_X, local_Y);
            rover(i).collectHeight(local_Z, mask);
        end
    end

    % Update plot
    drawnow limitrate;
end
time = toc;

% Process collected height data from both rovers
combined_height_data = [rover(1).HeightData; rover(2).HeightData];

grid_resolution = floor(grid_size / (2 * plow_radius));
avg_height_grid = zeros(grid_resolution, grid_resolution);
grid_count = zeros(grid_resolution, grid_resolution);

for i = 1:size(combined_height_data, 1)
    x = combined_height_data(i, 1);
    y = combined_height_data(i, 2);
    avg_height = combined_height_data(i, 4);
    
    grid_x = floor(x / (2*plow_radius)) + 1;
    grid_y = floor(y / (2*plow_radius)) + 1;
    
    grid_x = max(1, min(grid_x, grid_resolution));
    grid_y = max(1, min(grid_y, grid_resolution));
    
    avg_height_grid(grid_y, grid_x) = avg_height_grid(grid_y, grid_x) + avg_height;
    grid_count(grid_y, grid_x) = grid_count(grid_y, grid_x) + 1;
end

% Calculate final average height for each grid cell
valid_cells = grid_count > 0;
avg_height_grid(valid_cells) = avg_height_grid(valid_cells) ./ grid_count(valid_cells);

% Plot the average height grid
figure;
imagesc([0 10], [0 10], avg_height_grid);
title('Combined Average Height Grid');
xlabel('X (meters)');
ylabel('Y (meters)');
axis equal;
set(gca, 'YDir', 'normal');
colorbar;
