classdef Rover < handle
    properties
        Position % [x, y, z]
        Dimensions % [width, height, depth]
        Speed
        PlowRadius
        Patch % Handle to the rover's patch object
        HeightData % Store collected height data
        Vertices % Store the rover's vertices
        Faces % Store the rover's faces
        Path % Store the path for the rover to follow
        CurrentPathIndex % Keep track of the current position in the path
    end
    
    methods
        function obj = Rover(initial_position, dimensions, speed, plow_radius)
            obj.Position = initial_position;
            obj.Dimensions = dimensions;
            obj.Speed = speed;
            obj.PlowRadius = plow_radius;
            obj.HeightData = [];
            [obj.Vertices, obj.Faces] = obj.generateRoverGeometry();
            obj.Patch = patch('Vertices', obj.Vertices, 'Faces', obj.Faces, ...
                'FaceColor', 'red', 'EdgeColor', 'black', 'FaceAlpha', 0.7);
            obj.CurrentPathIndex = 1;
            obj.updateVisualization();
        end
        
        function setPath(obj, path)
            obj.Path = path;
            obj.CurrentPathIndex = 1;
        end
        
        function move(obj)
            if obj.CurrentPathIndex <= size(obj.Path, 1)
                obj.Position = obj.Path(obj.CurrentPathIndex, :);
                obj.CurrentPathIndex = obj.CurrentPathIndex + 1;
                obj.updateVisualization();
            end
        end
        
        function updateVisualization(obj)
            transformed_vertices = obj.Vertices + obj.Position - [obj.Dimensions(1)/2, obj.Dimensions(2)/2, 0];
            set(obj.Patch, 'Vertices', transformed_vertices, ...
                'AmbientStrength', 0.8, ...
                'DiffuseStrength', 0.2, ...
                'SpecularStrength', 0);
        end

        
        function [vertices, faces] = generateRoverGeometry(obj)
            width = obj.Dimensions(1);
            height = obj.Dimensions(2);
            depth = obj.Dimensions(3);
            
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
        
        function collectHeight(obj, local_Z, mask)
            avg_height = mean(local_Z(mask));
            obj.HeightData(end+1, :) = [obj.Position, avg_height];
        end

        
        function hasNextPosition(obj)
            obj.CurrentPathIndex <= size(obj.Path, 1);
        end
    end
end
