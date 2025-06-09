% Clear workspace and close all figures
clear all;
close all;

% Create a 5x5 meter grid with high resolution
[X, Y] = meshgrid(0:0.01:5, 0:0.01:5);

% Generate base terrain using multiple octaves of random noise
Z = zeros(size(X));
octaves = 4;
persistence = 0.5;
for i = 1:octaves
    freq = 2^i;
    amp = persistence^(i-1);
    Z = Z + amp * interp2(randn(freq+1), linspace(1,freq+1,size(X,1)), linspace(1,freq+1,size(X,2)), 'cubic');
end

% Scale the terrain to maintain the desired height range
Z = 0.03 * Z / max(abs(Z(:)));

% Add some random bumps of varying heights
num_bumps = 50;
for i = 1:num_bumps
    x = rand() * 5;
    y = rand() * 5;
    height = (0.005 + 0.025 * rand()) * randn();  % Random height between 0.5 to 3 cm
    width = 0.1 + 0.2 * rand();  % Random width between 10 to 30 cm
    Z = Z + height * exp(-((X-x).^2 + (Y-y).^2) / (2*width^2));
end

% Add some larger, gentle slopes
[Xm, Ym] = meshgrid(linspace(0,1,size(X,1)), linspace(0,1,size(X,2)));
Z = Z + 0.2 * (Xm + Ym);

% Ensure the terrain is always positive
Z = Z - min(Z(:));

% Apply a slight Gaussian filter to smooth very sharp transitions
Z = imgaussfilt(Z, 1);

% Initialize plow count matrix
plow_count = zeros(size(Z));

% Create full screen figure with 2D view
figure('units','normalized','outerposition',[0 0 1 1]); % Full screen
surf(X, Y, Z, 'EdgeColor', 'none');
hold on;

% Configure 2D view settings
view(2); % Top-down view
axis equal tight
axis([0 5 0 5]);
set(gca,'XTick',[],'YTick',[],'XColor','none','YColor','none'); % Remove axes

% Rover dimensions
width = 0.3; height = 0.15; depth = 0.3;

% Generate initial rover
[vertices, faces] = generateRover(width, height, depth);

% Create initial rover patch
roverPatch = patch('Vertices', vertices, 'Faces', faces, ...
    'FaceColor', 'red', 'EdgeColor', 'black', 'FaceAlpha', 0.7);

% Define rover speed (meters per second)
rover_speed = 0.1;

% Generate spiral plowing path
spiral_resolution = 100;
t = linspace(0, 2*pi, spiral_resolution);
spiral_turns = 10;
a = 2.5; % Scale factor to fit the 5x5 grid
b = 2.5; % Offset to center the spiral

path_x = [];
path_y = [];

for i = 1:spiral_turns
    r = (spiral_turns - i + 1) / spiral_turns * 2.5;
    x = r * cos(t) + b;
    y = r * sin(t) + b;
    path_x = [path_x, x];
    path_y = [path_y, y];
end

% Clip the path to stay within the 5x5 grid
path_x = max(0, min(5, path_x));
path_y = max(0, min(5, path_y));

% Interpolate Z values for the path
path_z = interp2(X, Y, Z, path_x, path_y);

% Plot the path
plot3(path_x, path_y, path_z, 'r-', 'LineWidth', 2);

% Calculate total path length
path_length = sum(sqrt(diff(path_x).^2 + diff(path_y).^2 + diff(path_z).^2));

% Calculate time step based on rover speed and path resolution
dt = rover_speed / (path_length / length(path_x));

% Animate rover along the path
for i = 1:length(path_x)
    % Get current position
    x = path_x(i);
    y = path_y(i);
    z = path_z(i);

    % Calculate 2D gradient
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

    % Update terrain (plowing effect)
    [~, ix] = min(abs(X(1,:) - x));
    [~, iy] = min(abs(Y(:,1) - y));
    plow_radius = 0.2;  % Radius of plowing effect
    [local_X, local_Y] = meshgrid(-plow_radius:0.01:plow_radius, -plow_radius:0.01:plow_radius);
    mask = (local_X.^2 + local_Y.^2) <= plow_radius^2;

    for dx = -plow_radius:0.01:plow_radius
        for dy = -plow_radius:0.01:plow_radius
            if dx^2 + dy^2 <= plow_radius^2
                local_ix = ix + round(dx / 0.01);
                local_iy = iy + round(dy / 0.01);
                if local_ix > 0 && local_ix <= size(Z, 2) && local_iy > 0 && local_iy <= size(Z, 1)
                    plow_count(local_iy, local_ix) = plow_count(local_iy, local_ix) + 1;

                    % Calculate the target height (50% of original height)
                    target_height = 0.5 * Z(local_iy, local_ix);

                    % Gradually reduce height towards target
                    Z(local_iy, local_ix) = Z(local_iy, local_ix) - (Z(local_iy, local_ix) - target_height) * 0.3;
                end
            end
        end
    end

    % Update terrain plot
    set(surf_handle, 'ZData', Z);

    % Update colorbar title with plow count
    set(colorbar_title, 'String', sprintf('Elevation (m)\nMax Plow Count: %d', max(plow_count(:))));

    % Update plot
    drawnow;
end

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