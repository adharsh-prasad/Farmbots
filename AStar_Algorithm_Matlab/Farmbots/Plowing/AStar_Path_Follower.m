% Clear workspace and close all figures
close all;

% Create a 5x5 meter grid with high resolution
[X, Y] = meshgrid(0:0.01:5, 0:0.01:5);

% Base sinusoidal terrain with varying amplitude and frequency
amplitude_variation = 0.1 * rand(size(X));
frequency_variation_x = 2 * pi * (0.1 * rand(size(X)));
frequency_variation_y = 2 * pi * (0.1 * rand(size(Y)));

% Generate the base sinusoidal terrain
Z = amplitude_variation .* sin(frequency_variation_x .* X) .* cos(frequency_variation_y .* Y);

% Add random bumps for more realism
num_bumps = 2000;
for i = 1:num_bumps
    x_center = rand() * 5;
    y_center = rand() * 5;
    height = 1*(0.005 + 0.025 * rand()) * randn();
    width = 0.1 + 0.2 * rand();
    Z = Z + height * exp(-((X - x_center).^2 + (Y - y_center).^2) / (2 * width^2));
end

% Ensure the terrain is always positive
Z = (abs(Z));

% Apply Gaussian smoothing multiple times for smoother terrain
Z = imgaussfilt(Z, 1);  % First pass smoothing
Z = imgaussfilt(Z, 1);  % Second pass for more smoothing

% Plot the terrain
figure;
surf_handle = surf(X, Y, Z, 'EdgeColor', 'none');
hold on;

title('Simulated Agricultural Field with Rover Path');
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Elevation (meters)');
axis([0 5 0 5 0 5]);

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

% Set start position
start_pos = [1, 1];

% Get optimal leveling path using A*
[path, visit_count] = multi_robot_terrain_leveling_astar(avg_height_grid, start_pos);

% Convert path indices to actual coordinates
path_x = (path(:,2) - 1) * (2 * plow_radius);
path_y = (path(:,1) - 1) * (2 * plow_radius);
path_z = interp2(X, Y, Z, path_x, path_y);

% % Interpolate the path for smoother rover movement
% smooth_path_x = interp1(1:length(path_x), path_x, linspace(1, length(path_x), 500), 'spline');
% smooth_path_y = interp1(1:length(path_y), path_y, linspace(1, length(path_y), 500), 'spline');
% smooth_path_z = interp2(X, Y, Z, smooth_path_x, smooth_path_y);
% 
% % Calculate path length
% path_length = sum(sqrt(diff(smooth_path_x).^2 + diff(smooth_path_y).^2 + diff(smooth_path_z).^2));
% 
% % Calculate time step based on rover speed and path resolution
% dt = rover_speed / (path_length / length(smooth_path_x));
% 
% % Initialize visualization for collected data points
% plow_count = zeros(size(Z));
% 
% % Animate rover along the path with reduced plot updates
% for i = 1:length(smooth_path_x)
%     if mod(i, 5) == 0  % Update plot every 5 frames
%         % Update terrain plot
%         set(surf_handle, 'ZData', Z);
% 
%         % Update plot
%         drawnow;
%     end
% 
%     % Get current position
%     x = smooth_path_x(i);
%     y = smooth_path_y(i);
%     z = smooth_path_z(i);
% 
%     % Calculate 2D gradient for rover orientation
%     [fx, fy] = gradient(Z);
%     gx = interp2(X, Y, fx, x, y);
%     gy = interp2(X, Y, fy, x, y);
% 
%     % Calculate surface normal
%     normal = [-gx, -gy, 1];
%     normal = normal / norm(normal);
% 
%     % Calculate forward direction
%     if i < length(smooth_path_x)
%         forward = [smooth_path_x(i+1) - x, smooth_path_y(i+1) - y, smooth_path_z(i+1) - z];
%     else
%         forward = [smooth_path_x(i) - smooth_path_x(i-1), smooth_path_y(i) - smooth_path_y(i-1), smooth_path_z(i) - smooth_path_z(i-1)];
%     end
%     forward = forward / norm(forward);
% 
%     % Calculate right vector
%     right = cross(forward, normal);
%     right = right / norm(right);
% 
%     % Recalculate up vector
%     up = cross(right, forward);
% 
%     % Create rotation matrix
%     R = [right', forward', up'];
% 
%     % Apply rotation and translation to vertices
%     transformed_vertices = (R * (vertices - [width/2, depth/2, height/2])')' + [x, y, z];
% 
%     % Update rover plot
%     set(roverPatch, 'Vertices', transformed_vertices);
% 
%     % Update terrain modification based on visit count
%     [~, ix] = min(abs(X(1,:) - x));
%     [~, iy] = min(abs(Y(:,1) - y));
% 
%     % Calculate required passes for current location
%     local_height = Z(iy, ix);
%     required_passes = ceil((local_height - target_height) / 0.05);
% 
%     % Modify plowing effect based on required passes
%     for dx = -plow_radius:0.01:plow_radius
%         for dy = -plow_radius:0.01:plow_radius
%             if dx^2 + dy^2 <= plow_radius^2
%                 local_ix = ix + round(dx / 0.01);
%                 local_iy = iy + round(dy / 0.01);
%                 if local_ix > 0 && local_ix <= size(Z, 2) && local_iy > 0 && local_iy <= size(Z, 1)
%                     current_passes = plow_count(local_iy, local_ix);
%                     if current_passes < required_passes
%                         plow_count(local_iy, local_ix) = current_passes + 1;
% 
%                         % Adjust height reduction based on number of passes
%                         reduction_factor = 0.3 * (required_passes - current_passes) / required_passes;
%                         Z(local_iy, local_ix) = Z(local_iy, local_ix) - ...
%                             (Z(local_iy, local_ix) - target_height) * reduction_factor;
%                     end
%                 end
%             end
%         end
%     end
%     pause(0.1)
% end

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
