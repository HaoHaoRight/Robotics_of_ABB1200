classdef cost
    % [II.B Definition 1-5] 成本函数
    % c(x,y) | 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。
    % c_(x,y) | 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
    % gT(x) | 计算根节点通过树Tree到X空间内点x的已发生成本。
    % g_(x) | 为X空间内点x已经发生的下界估计。
    % h_(x) | 为X空间内点x的启发式的下界估计。h_(x,y):=c_(x,x_t)，其中x_t是通过规划问题找到的最佳解决方案的终点。

    methods   
       
 % c(x,y) | 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。               
       function pathLength = c(x, y, obstacle)
            % x:空间一点，y：空间一点，obstacle：障碍物类
            % 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。
            if obstacle.isVectorIntersectingObstacle(x, y)
                pathLength = Inf; % 如果相交，返回无穷大
                return;
            end
            pathLength = norm(x - y); % 计算路径长度
       end
       
% c_(x,y) | 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
       function pathLengthlowCost = c_(x, y)
            % 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。
            % 使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
            pathLengthlowCost = norm(x - y); % 使用欧几里得距离作为路径成本的下界估计
       end
       
% gT(x) | 计算根节点通过树Tree到X空间内点x的已发生成本。    
       function gT = gT(current_node, Tree)
            % 获取从起始节点到指定节点的路径代价
            % current_node: 要计算其代价的节点，Tree: 树
            
            % 如果节点是起始节点，则代价为 0。
            if isequal(current_node,Tree.V(1))
                gT = 0;
            % 如果节点不在树中，则代价为无穷大。
            elseif ~ismember(current_node,Tree.V)
                gT = Inf;
            % 返回通过树遍历从起始节点到指定节点的路径代价。
            else
                % 递归计算从起始节点到父节点的路径代价。
                for i = 1:length(Tree.V)
                        % 从树中获取指定节点的父节点。
                        father = Tree.E{Tree.V == current_node}.edge(1);
                        gT = gT + Tree.E{Tree.V == current_node}.cost;% ?不确定
                        current_node = father;
                end
            end
       end
        
       
% g_(x) | 为X空间内点x已经发生的下界估计。   
        function g_hat = g_(current_node,start)
            % current_node是当前节点，start是起始节点
            % 生成当前节点的 g_hat 值。这是当前节点与起始节点之间的 L2 范数。
            % 计算当前节点与起始节点之间的 L2 范数。
            g_hat = norm(current_node - start);
        end
        
 % h_(x) | 为X空间内点x的启发式的下界估计。h_(x,y):=c_(x,x_t)，其中x_t是通过规划问题找到的最佳解决方案的终点。
        function h_hat = h_(current_node,goal)
            %current_node是当前节点，goal是目标节点             
            % 生成当前节点的 h_hat 值。这是当前节点与目标节点之间的 L2 范数。
            % 计算当前节点与目标节点之间的 L2 范数。
            h_hat = norm(current_node - goal);
        end
        
% 定义 11 (cost ← BestValue(Qi))：查找 Qi 中队列成本最低的元素，并返回该元素的队列成本。    
% Definition 11  (cost ← BestValue(Qi)) 
% Finds the elementin Qi with the lowest queue cost and returns the queue cost of that element.
        function BestValue = BestValue(Q, name, Tree)
            % Q:队列，name:队列名称

            % QV queue cost: gT(v)+h_(v)
            if(name == 'V')
                BestValue = min(cost.gT(Q.V, Tree)+cost.h_(Q.V));
            end
            % QE queue cost: gT(v)+c_(v, x)+h_(x)
            if(name == 'E')
                BestValue = min(cost.gT(Q.E.edge(1), Tree)+cost.c_(Q.E.edge(1), Q.E.edge(2))+cost.h_(Q.E.edge(2)));
            end

        end  

    end % methods
end % classdef