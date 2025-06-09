% Clear workspace and close all figures 
close all; 
avg_height_grid = imresize(Z, [grid_resolution, grid_resolution]);

% Create a fullscreen figure without borders or menus
fig = figure('WindowState', 'fullscreen', 'MenuBar', 'none', 'ToolBar', 'none', 'NumberTitle', 'off', 'Name', '', 'Color', 'k');

% Maximize the figure window to remove any gaps
set(fig, 'WindowStyle', 'normal');

% Create a high-resolution 5x5 meter grid
[X, Y] = meshgrid(0:0.01:10, 0:0.01:10);

% Generate the terrain (using the same code as before)
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

% Set axis properties, lighting, and colormap (same as before)
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

% Rover dimensions
width = 0.3; height = 0.15; depth = 0.3;

% Define rover parameters
rover_speed = 0.1;
plow_radius = 0.1;

% Calculate grid resolution based on plow radius
grid_resolution = floor(5 / (2 * plow_radius));
cell_size = 5 / grid_resolution;

% Define number of robots and their start positions
num_robots = 4;
start_positions = [1, 1; 1, grid_resolution; grid_resolution, 1; grid_resolution, grid_resolution];

% Get optimal leveling paths using multi-robot A*
[paths, visit_count] = multi_robot_terrain_leveling(avg_height_grid, start_positions, num_robots);

% Initialize rover patches and smooth paths for each robot
roverPatches = cell(1, num_robots);
smooth_paths = cell(1, num_robots);

for r = 1:num_robots
    [vertices, faces] = generateRover(width, height, depth);
    roverPatches{r} = patch('Vertices', vertices, 'Faces', faces, 'FaceColor', 'red', 'EdgeColor', 'black', 'FaceAlpha', 0.7);
    
    path = paths{r};
    path_x = (path(:,2) - 0.5) * cell_size;
    path_y = (path(:,1) - 0.5) * cell_size;
    
    smooth_path_x = max(0, min(5, path_x));
    smooth_path_y = max(0, min(5, path_y));
    smooth_path_z = interp2(X, Y, Z, smooth_path_x, smooth_path_y);
    
    num_points = 500;
    smooth_paths{r}.x = interp1(1:length(smooth_path_x), smooth_path_x, linspace(1, length(smooth_path_x), num_points), 'spline');
    smooth_paths{r}.y = interp1(1:length(smooth_path_y), smooth_path_y, linspace(1, length(smooth_path_y), num_points), 'spline');
    smooth_paths{r}.z = interp2(X, Y, Z, smooth_paths{r}.x, smooth_paths{r}.y);
end

% Calculate time step based on rover speed and longest path
max_path_length = max(cellfun(@(p) sum(sqrt(diff(p.x).^2 + diff(p.y).^2 + diff(p.z).^2)), smooth_paths));
dt = rover_speed / (max_path_length / num_points);

% Initialize visualization for collected data points
plow_count = zeros(size(Z));

% Animate rovers along their paths
for i = 1:num_points
    if mod(i, 5) == 0
        set(surf_handle, 'ZData', Z);
        drawnow;
    end

    for r = 1:num_robots
        % Get current position
        x = smooth_paths{r}.x(i);
        y = smooth_paths{r}.y(i);
        z = smooth_paths{r}.z(i);

        % Calculate 2D gradient for rover orientation
        [fx, fy] = gradient(Z);
        gx = interp2(X, Y, fx, x, y);
        gy = interp2(X, Y, fy, x, y);

        % Calculate surface normal
        normal = [-gx, -gy, 1];
        normal = normal / norm(normal);

        % Calculate forward direction
        if i < num_points
            forward = [smooth_paths{r}.x(i+1) - x, smooth_paths{r}.y(i+1) - y, smooth_paths{r}.z(i+1) - z];
        else
            forward = [smooth_paths{r}.x(i) - smooth_paths{r}.x(i-1), smooth_paths{r}.y(i) - smooth_paths{r}.y(i-1), smooth_paths{r}.z(i) - smooth_paths{r}.z(i-1)];
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
        [vertices, ~] = generateRover(width, height, depth);
        transformed_vertices = (R * (vertices - [width/2, depth/2, height/2])')' + [x, y, z];

        % Update rover plot
        set(roverPatches{r}, 'Vertices', transformed_vertices);

        % Update terrain modification
        [~, ix] = min(abs(X(1,:) - x));
        [~, iy] = min(abs(Y(:,1) - y));

        % Calculate required passes
        local_height = Z(iy, ix);
        required_passes = ceil((local_height - 0) / 0.05);

        % Modify terrain
        for dx = -plow_radius:0.01:plow_radius
            for dy = -plow_radius:0.01:plow_radius
                if dx^2 + dy^2 <= plow_radius^2
                    local_ix = ix + round(dx / 0.01);
                    local_iy = iy + round(dy / 0.01);
                    if local_ix > 0 && local_ix <= size(Z, 2) && local_iy > 0 && local_iy <= size(Z, 1)
                        current_passes = plow_count(local_iy, local_ix);
                        if current_passes < required_passes
                            plow_count(local_iy, local_ix) = current_passes + 1;
                            reduction_factor = 0.3 * (required_passes - current_passes) / required_passes;
                            Z(local_iy, local_ix) = Z(local_iy, local_ix) - (Z(local_iy, local_ix) - 0) * reduction_factor;
                        end
                    end
                end
            end
        end
    end
    pause(0.01)
end