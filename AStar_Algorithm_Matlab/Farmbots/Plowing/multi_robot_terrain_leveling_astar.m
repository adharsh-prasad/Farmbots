function [paths, leveledMap] = multi_robot_terrain_leveling_astar(map, startPositions, goalPositions)
    [rows, cols] = size(map);
    numRobots = length(startPositions);
    paths = cell(1, numRobots);
    
    % Initialize priority queue for each robot
    openSets = cell(1, numRobots);
    for i = 1:numRobots
        openSets{i} = PriorityQueue();
        openSets{i}.push(startPositions(i,:), 0);
    end
    
    % Initialize closed sets
    closedSets = cell(1, numRobots);
    for i = 1:numRobots
        closedSets{i} = zeros(rows, cols);
    end
    
    % Initialize parent maps
    parentMaps = cell(1, numRobots);
    for i = 1:numRobots
        parentMaps{i} = zeros(rows, cols, 2);
    end
    
    % Main loop
    while ~all(cellfun(@isempty, openSets))
        for r = 1:numRobots
            if ~isempty(openSets{r})
                [current, ~] = openSets{r}.pop();
                
                if isequal(current, goalPositions(r,:))
                    paths{r} = reconstructPath(parentMaps{r}, current);
                    continue;
                end
                
                closedSets{r}(current(1), current(2)) = 1;
                
                % Generate neighbors
                neighbors = generateNeighbors(current, rows, cols);
                
                for i = 1:size(neighbors, 1)
                    neighbor = neighbors(i,:);
                    
                    if closedSets{r}(neighbor(1), neighbor(2)) || map(neighbor(1), neighbor(2)) == 1
                        continue;
                    end
                    
                    tentativeG = norm(neighbor - startPositions(r,:));
                    
                    if ~openSets{r}.contains(neighbor) || tentativeG < openSets{r}.getPriority(neighbor)
                        parentMaps{r}(neighbor(1), neighbor(2),:) = current;
                        h = norm(neighbor - goalPositions(r,:));
                        f = tentativeG + h;
                        openSets{r}.push(neighbor, f);
                    end
                end
            end
        end
    end
    
    % Level the land
    leveledMap = levelLand(map, paths);
end

function neighbors = generateNeighbors(current, rows, cols)
    offsets = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
    neighbors = current + offsets;
    neighbors = neighbors(all(neighbors > 0, 2) & neighbors(:,1) <= rows & neighbors(:,2) <= cols, :);
end

function path = reconstructPath(parentMap, goal)
    path = goal;
    current = goal;
    while any(parentMap(current(1), current(2),:))
        current = squeeze(parentMap(current(1), current(2),:))';
        path = [current; path];
    end
end

function leveledMap = levelLand(map, paths)
    leveledMap = map;
    for i = 1:length(paths)
        for j = 1:size(paths{i}, 1)
            current = paths{i}(j,:);
            leveledMap(current(1), current(2)) = 0;  % Set to level ground
        end
    end
end

% Priority Queue implementation (simplified)
classdef PriorityQueue < handle
    properties (Access = private)
        elements
        priorities
    end
    
    methods
        function obj = PriorityQueue()
            obj.elements = {};
            obj.priorities = [];
        end
        
        function push(obj, element, priority)
            obj.elements{end+1} = element;
            obj.priorities(end+1) = priority;
        end
        
        function [element, priority] = pop(obj)
            [~, idx] = min(obj.priorities);
            element = obj.elements{idx};
            priority = obj.priorities(idx);
            obj.elements(idx) = [];
            obj.priorities(idx) = [];
        end
        
        function tf = isempty(obj)
            tf = isempty(obj.elements);
        end
        
        function tf = contains(obj, element)
            tf = any(cellfun(@(x) isequal(x, element), obj.elements));
        end
        
        function priority = getPriority(obj, element)
            idx = find(cellfun(@(x) isequal(x, element), obj.elements), 1);
            if isempty(idx)
                priority = Inf;
            else
                priority = obj.priorities(idx);
            end
        end
    end
end