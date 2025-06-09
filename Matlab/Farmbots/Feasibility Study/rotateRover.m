function rotateRover()
    % Create a blank figure
    figure;
    hold on;
    axis equal;
    axis([-1 1 -1 1 -1 1]);  % Set axis limits
    view(3);  % Set 3D view
    
    % Rover dimensions
    width = 0.3; height = 0.15; depth = 0.3;
    
    % Generate initial rover
    [vertices, faces] = generateRover(width, height, depth);
    
    % Plot initial rover
    roverPatch = patch('Vertices', vertices, 'Faces', faces, ...
        'FaceColor', 'red', 'EdgeColor', 'black', 'FaceAlpha', 0.7);

    % Create title and subtitle
    title('Rover Rotation Simulation');
    subTitle = subtitle('');

    % Rotate rover
    for angle = 0:10:360
        % Create rotation matrix
        angleX = angle;
        angleY = angle/2;
        angleZ = angle/3;
        Rx = makehgtform('xrotate', deg2rad(angleX));
        Ry = makehgtform('yrotate', deg2rad(angleY));
        Rz = makehgtform('zrotate', deg2rad(angleZ));
        
        % Combine rotations
        R = Rx * Ry * Rz;
        
        % Apply rotation to vertices
        rotatedVertices = (R(1:3, 1:3) * vertices')';
        
        % Update rover plot
        set(roverPatch, 'Vertices', rotatedVertices);
        
        % Update subtitle with current angles
        set(subTitle, 'String', sprintf('X: %.1f°, Y: %.1f°, Z: %.1f°', angleX, angleY, angleZ));
        
        % Update plot
        drawnow;
        pause(0.1);
    end
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
    ] - [width/2, depth/2, height/2];  % Center the rover at origin

    faces = [1 2 3 4
            5 6 7 8
            4 3 7 8
            3 2 6 7
            2 1 5 6
            1 4 8 5];
end