classdef Obstacle
    properties
        obstacles = {}; % 障碍物列表
    end
    
    methods
        function obj = addObstacle(obj, obstacle)
            % 添加障碍物到场景
            %   obstacle: 要添加的障碍物对象
            obj.obstacles{end+1} = obstacle;
        end
        
        function minDist = distanceToPoint(obj, point)
            % 计算场景中所有障碍物到指定点的最小距离
            %   point: 指定点的坐标
            minDist = Inf;
            for i = 1:numel(obj.obstacles)
                obstacle = obj.obstacles{i};
                dist = obstacle.distanceToPoint(point);
                minDist = min(minDist, dist);
            end
        end
        
        function plotScene(obj)
            % 绘制场景中的所有障碍物
            for i = 1:numel(obj.obstacles)
                obstacle = obj.obstacles{i};
                obstacle.plotObstacle();
            end
        end

        function intersecting = isVectorIntersectingObstacle(obj, vectorStart, vectorEnd)
            % 判断向量是否与场景中的任何障碍物相交
            %   vectorStart: 向量的起点坐标
            %   vectorEnd: 向量的终点坐标
            % 返回 true 如果相交，否则返回 false
            intersecting = false;
            for i = 1:numel(obj.obstacles)
                obstacle = obj.obstacles{i};
                if obstacle.isVectorIntersectingObstacle(vectorStart, vectorEnd)
                    intersecting = true;
                    break;
                end
            end
        end
        
        function inside = isPointInside(obj, point)
            % 判断点是否在场景中的任何障碍物内
            %   point: 要检查的点的坐标 [x, y, z]
            inside = false;
            for i = 1:numel(obj.obstacles)
                obstacle = obj.obstacles{i};
                if obstacle.isPointInside(point)
                    inside = true;
                    break;
                end
            end
        end

        function plotObstacle(obj)
            for i = 1:numel(obj.obstacles)
                obstacle = obj.obstacles{i};
                obstacle.plotObstacle();
            end
        end
    end
end