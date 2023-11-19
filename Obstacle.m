classdef Obstacle
    properties
        vertices; % 障碍物顶点
        edges = [ % 立方体的边是固定的
            1 2; 2 3; 3 4; 4 1; % 底面
            5 6; 6 7; 7 8; 8 5; % 顶面
            1 5; 2 6; 3 7; 4 8  % 侧面
        ];
    end
    
    methods
        function obj = Obstacle(center, length, width, height)
            % 计算立方体的顶点
            l = length / 2;
            w = width / 2;
            h = height / 2;
            obj.vertices = [
                center + [-l -w -h];
                center + [-l w -h];
                center + [l w -h];
                center + [l -w -h];
                center + [-l -w h];
                center + [-l w h];
                center + [l w h];
                center + [l -w h]
            ];
        end
        
        function minDist = distanceToPoint(obj, point)
            % 确定立方体的边界
            minCorner = min(obj.vertices);
            maxCorner = max(obj.vertices);
            
            % 计算每个维度上的距离
            dx = max(max(minCorner(1) - point(1), 0), point(1) - maxCorner(1));
            dy = max(max(minCorner(2) - point(2), 0), point(2) - maxCorner(2));
            dz = max(max(minCorner(3) - point(3), 0), point(3) - maxCorner(3));
        
            % 计算欧几里得距离
            minDist = sqrt(dx^2 + dy^2 + dz^2);
        end
        
        function plotObstacle(obj)
            % 绘制障碍物
            for edge = obj.edges'
                plot3([obj.vertices(edge(1), 1), obj.vertices(edge(2), 1)], ...
                      [obj.vertices(edge(1), 2), obj.vertices(edge(2), 2)], ...
                      [obj.vertices(edge(1), 3), obj.vertices(edge(2), 3)], 'k-', 'LineWidth', 2);
            end
        end
    end
end