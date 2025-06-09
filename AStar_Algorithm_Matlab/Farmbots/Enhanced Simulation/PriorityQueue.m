% Simple priority queue implementation
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
        
        function element = pop(obj)
            [~, idx] = min(obj.priorities);
            element = obj.elements{idx};
            obj.elements(idx) = [];
            obj.priorities(idx) = [];
        end
        
        function result = isEmpty(obj)
            result = isempty(obj.elements);
        end
    end
end
