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
[X, Y] = meshgrid(0:0.01:5, 0:0.01:5);

% Generate the terrain using sinusoidal variations
amplitude_variation = 0.1 * rand(size(X));
frequency_variation_x = 2 * pi * (0.1 * rand(size(X)));
frequency_variation_y = 2 * pi * (0.1 * rand(size(Y)));
Z = amplitude_variation .* sin(frequency_variation_x .* X) .* cos(frequency_variation_y .* Y);

% Add bumps for texture
num_bumps = 2000;
for i = 1:num_bumps
    x_center = rand() * 5;
    y_center = rand() * 5;
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
xlim([0 5]); 
ylim([0 5]);

% Set lighting and colormap
lighting phong;
camlight('headlight');
shading interp;
colormap(parula);

% Remove white space from the figure
set(gca, 'Position', [0 0 1 1]);

% Rover dimensions
width = 0.3; height = 0.15; depth = 0.3;

% Generate initial rover
[vertices, faces] = generateRover(width, height, depth);

% Create initial rover patch
roverPatch = patch('Vertices', vertices, 'Faces', faces, ...
    'FaceColor', 'red', 'EdgeColor', 'black', 'FaceAlpha', 0.7);

% Define rover parameters
rover_speed = 0.1;
plow_radius = 0.1;
grid_size = 5;

% Calculate grid resolution based on plow diameter
grid_resolution = floor(grid_size / (2 * plow_radius));
avg_height_grid = zeros(grid_resolution, grid_resolution);
grid_count = zeros(grid_resolution, grid_resolution);

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

% Interpolate Z values for the path
path_z = interp2(X, Y, Z, path_x, path_y);

% Initialize collected data plot handle
collected_points_handle = [];

tic
% Animate rover along the path and collect data
for i = 1:length(path_x)
    % Get current position
    x = path_x(i);
    y = path_y(i);
    z = path_z(i);

    % Calculate grid indices
    grid_x = floor(x / (2*plow_radius)) + 1;
    grid_y = floor(y / (2*plow_radius)) + 1;

    % Ensure grid indices are within bounds
    grid_x = max(1, min(grid_x, grid_resolution));
    grid_y = max(1, min(grid_y, grid_resolution));

    % Collect height data
    [local_X, local_Y] = meshgrid(x-plow_radius:0.01:x+plow_radius, y-plow_radius:0.01:y+plow_radius);
    mask = (local_X - x).^2 + (local_Y - y).^2 <= plow_radius^2;
    local_Z = interp2(X, Y, Z, local_X, local_Y);
    avg_height = mean(local_Z(mask));

    % Update average height grid
    avg_height_grid(grid_y, grid_x) = avg_height_grid(grid_y, grid_x) + avg_height;
    grid_count(grid_y, grid_x) = grid_count(grid_y, grid_x) + 1;

    % Calculate 2D gradient for rover orientation
    [fx, fy] = gradient(Z);
    gx = interp2(X, Y, fx, x, y);
    gy = interp2(X, Y, fy, x, y);

    % Calculate surface normal
    normal = [-gx, -gy, 1];
    normal = normal / norm(normal);

    % Calculate forward direction
    if i < length(path_x)
        forward = [path_x(i+1) - x, path_y(i+1) - y, path_z(i+1) - z];
    else
        forward = [path_x(i) - path_x(i-1), path_y(i) - path_y(i-1), path_z(i) - path_z(i-1)];
    end
    forward = forward / norm(forward);

    % Calculate right vector
    right = cross(forward, normal);
    right = right / norm(right);

    % Recalculate up vector
    up = cross(right, forward);

    % Create rotation matrix
    R = [right', forward', up'];

    % Apply rotation and translation to vertices
    transformed_vertices = (R * (vertices - [width/2, depth/2, height/2])')' + [x, y, z];

    % Update rover plot
    set(roverPatch, 'Vertices', transformed_vertices);
    % collected_points_handle = scatter3(path_x(1:i), path_y(1:i), 0.25, "o", "filled", 'g');

    % Update plot
    drawnow limitrate;
end
time = toc;
% Calculate final average height for each grid cell
valid_cells = grid_count > 0;
avg_height_grid(valid_cells) = avg_height_grid(valid_cells) ./ grid_count(valid_cells);

% Plot the average height grid
figure;
imagesc([0 5], [0 5], avg_height_grid);
title('Average Height Grid');
xlabel('X (meters)');
ylabel('Y (meters)');
axis equal;
set(gca, 'YDir', 'normal');

% Rover generation function
function [vertices, faces] = generateRover(width, height, depth)
    vertices = [
        0 0 0;         % Bottom face vertices
        width 0 0;
        width depth 0;
        0 depth 0;
        0 0 height;    % Top face vertices
        width 0 height;
        width depth height;
        0 depth height;
    ];

    faces = [1 2 3 4
            5 6 7 8
            4 3 7 8
            3 2 6 7
            2 1 5 6
            1 4 8 5];
end
