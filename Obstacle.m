% FILEPATH: /c:/Users/a1606/Documents/robot/Robotics_of_ABB1200/Obstacle.m
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
            % 创建一个障碍物对象
            %   center: 障碍物的中心点坐标
            %   length: 障碍物的长度
            %   width: 障碍物的宽度
            %   height: 障碍物的高度
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
            % 计算障碍物到指定点的最小距离
            %   point: 指定点的坐标
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

        function intersecting = isVectorIntersectingObstacle(obj, vectorStart, vectorEnd)
            % 判断向量与长方体障碍物是否相交
            %   vectorStart: 向量的起点坐标
            %   vectorEnd: 向量的终点坐标
            % 返回 true 如果相交，否则返回 false
            
            % 计算向量的参数表示式：x = x0 + t*(x1-x0), y = y0 + t*(y1-y0), z = z0 + t*(z1-z0)
            % 其中(x0, y0, z0)是起点坐标，(x1, y1, z1)是终点坐标
            x0 = vectorStart(1); y0 = vectorStart(2); z0 = vectorStart(3);
            x1 = vectorEnd(1); y1 = vectorEnd(2); z1 = vectorEnd(3);
            % 计算向量与正方体每个面的交点
            intersections = [
                obj.intersectionWithPlane(x0, y0, z0, x1, y1, z1, min(obj.vertices(:,1)), 'x');
                obj.intersectionWithPlane(x0, y0, z0, x1, y1, z1, max(obj.vertices(:,1)), 'x');
                obj.intersectionWithPlane(x0, y0, z0, x1, y1, z1, min(obj.vertices(:,2)), 'y');
                obj.intersectionWithPlane(x0, y0, z0, x1, y1, z1, max(obj.vertices(:,2)), 'y');
                obj.intersectionWithPlane(x0, y0, z0, x1, y1, z1, min(obj.vertices(:,3)), 'z');
                obj.intersectionWithPlane(x0, y0, z0, x1, y1, z1, max(obj.vertices(:,3)), 'z');
            ];
            
            % 检查交点是否在正方体面内
            intersecting = any(all(intersections >= min(obj.vertices) - eps & intersections <= max(obj.vertices) + eps, 2));
        end
        
        function intersection = intersectionWithPlane(obj, x0, y0, z0, x1, y1, z1, planeVal, planeAxis)
            % 计算向量与平行于坐标轴的平面的交点
            %   x0, y0, z0: 向量的起点坐标
            %   x1, y1, z1: 向量的终点坐标
            %   planeVal: 平面的坐标值（x、y 或 z）
            %   planeAxis: 平面所在的轴（'x'、'y' 或 'z'）
            % 返回交点坐标
            t = (planeVal - eval([planeAxis '0'])) / (eval([planeAxis '1']) - eval([planeAxis '0']));
            intersection = [x0 + t*(x1 - x0), y0 + t*(y1 - y0), z0 + t*(z1 - z0)];
        end
        
    end
end