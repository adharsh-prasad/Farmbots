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
Z = 0.2 * Z / max(abs(Z(:)));

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

% Plot the terrain
figure;
surf(X, Y, Z, 'EdgeColor', 'none');
hold on;

title('Simulated Agricultural Field with Rover Path');
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Elevation (meters)');
axis([0 5 0 5 0 5]);
colorbar;

% Generate plowing path
num_rows = 10;
path_x = [];
path_y = [];
for i = 0:num_rows
    if mod(i, 2) == 0
        path_x = [path_x, linspace(0, 5, 100)];
        path_y = [path_y, ones(1, 100) * i * 5/num_rows];
    else
        path_x = [path_x, linspace(5, 0, 100)];
        path_y = [path_y, ones(1, 100) * i * 5/num_rows];
    end
end

% Interpolate Z values for the path
path_z = interp2(X, Y, Z, path_x, path_y);

% Plot the path
plot3(path_x, path_y, path_z, 'r-', 'LineWidth', 2);

% Rover dimensions
width = 0.3; height = 0.15; depth = 0.3;

% Generate initial rover
[vertices, faces] = generateRover(width, height, depth);

% Create initial rover patch
roverPatch = patch('Vertices', vertices, 'Faces', faces, ...
    'FaceColor', 'red', 'EdgeColor', 'black', 'FaceAlpha', 0.7);

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
    
    % Update plot
    drawnow;
    pause(0.01);
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