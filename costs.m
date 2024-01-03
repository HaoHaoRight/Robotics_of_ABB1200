classdef costs
    % [II.B Definition 1-5] 成本函数
    % c(x,y) | 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。
    % c_(x,y) | 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
    % gT(x) | 计算根节点通过树Tree到X空间内点x的已发生成本。
    % g_(x) | 为X空间内点x已经发生的下界估计。
    % h_(x) | 为X空间内点x的启发式的下界估计。h_(x,y):=c_(x,x_t)，其中x_t是通过规划问题找到的最佳解决方案的终点。

    properties
        start;
        goal;
        obstacle;
    end

    methods
        function obj = costs(start, goal, obstacle)
            % 构造函数
            obj.start = start;
            obj.goal = goal;
            obj.obstacle = obstacle;
        end
 % c(x,y) | 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。               
        function pathLength = c(obj, x, y)
            % x:空间一点，y：空间一点，obstacle：障碍物类
            % 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。
            if obj.obstacle.isVectorIntersectingObstacle(x, y)
                pathLength = Inf; % 如果相交，返回无穷大
                return;
            end
            pathLength = norm(x - y); % 计算路径长度
       end
       
% c_(x,y) | 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
        function pathLengthlowCost = c_(obj, x, y)
            % 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。
            % 使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
            pathLengthlowCost = norm(x - y); % 使用欧几里得距离作为路径成本的下界估计
       end
       
% gT(x) | 计算根节点通过树Tree到X空间内点x的已发生成本。    
        function gT = gT(obj, current_node, Tree)
            % 计算从树的根节点到当前节点的路径成本。
            % current_node: 当前节点
            % Tree: 包含节点和边的树结构
            if isequal(current_node, Tree.V(1,:))
                gT = 0;
                return;
            end
            if isempty(current_node)
                gT = inf;
                return;
            end
            % 对 A 中的每个元素进行检查
            if ~ismember(current_node, Tree.V, 'rows')
                gT = inf;
                return;
            end
            parent_node = Tree.E.v(ismember(Tree.E.x, current_node, "rows"),:);
            
            % 计算从父节点到当前节点的成本。
            edge_cost = norm(current_node - parent_node);

            % 递归调用 gT 来计算从根节点到父节点的路径成本。
            parent_cost = obj.gT(parent_node, Tree);

            % 累加父节点的成本和当前边的成本。
            gT = parent_cost + edge_cost;
        end

% g_(x) | 为X空间内点x已经发生的下界估计。   
        function g_hat = g_(obj, current_node)
            % current_node是当前节点，start是起始节点
            % 生成当前节点的 g_hat 值。这是当前节点与起始节点之间的 L2 范数。
            % 计算当前节点与起始节点之间的 L2 范数。
            g_hat = norm(current_node - obj.start);
        end

        % h_(x) | 为X空间内点x的启发式的下界估计。h_(x,y):=c_(x,x_t)，其中x_t是通过规划问题找到的最佳解决方案的终点。
        function h_hat = h_(obj, current_node)
            %current_node是当前节点，goal是目标节点
            % 生成当前节点的 h_hat 值。这是当前节点与目标节点之间的 L2 范数。
            % 计算当前节点与目标节点之间的 L2 范数。
            h_hat = norm(current_node - obj.goal);
        end

        % 定义 11 (cost ← BestValue(Qi))：查找 Qi 中队列成本最低的元素，并返回该元素的队列成本。
        % Definition 11  (cost ← BestValue(Qi))
        % Finds the elementin Qi with the lowest queue cost and returns the queue cost of that element.
        function [BestValue, index] = BestValue(obj, Q, name, Tree)
            BestValue = []; % 初始化
            if strcmp(name, 'V')
                % 预分配数组大小
                BestValue = zeros(1, size(Q.V, 1));
                parfor i = 1:size(Q.V, 1)
                    BestValue(i) = obj.gT(Q.V(i,:), Tree) + obj.h_(Q.V(i,:));
                end
                if isempty(BestValue)
                    BestValue = inf;
                end
                [BestValue, index] = min(BestValue);
            end

            if strcmp(name, 'E')
                % 预分配数组大小
                BestValue = zeros(1, size(Q.E.v, 1));
                parfor i = 1:size(Q.E.v, 1)
                    BestValue(i) = obj.gT(Q.E.v(i,:), Tree) + obj.c_(Q.E.v(i,:), Q.E.x(i,:)) + obj.h_(Q.E.x(i,:));
                end
                if isempty(BestValue)
                    BestValue = inf;
                end
                [BestValue, index] = min(BestValue);
                index = index(1);
            end
        end

    end % methods
end % classdef