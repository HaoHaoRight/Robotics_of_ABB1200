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
       function pathLengthlowCost = pathcostlow(x, y)
            % 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。
            % 使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
            pathLengthlowCost = norm(x - y); % 使用欧几里得距离作为路径成本的下界估计
       end
       
% gT(x) | 计算根节点通过树Tree到X空间内点x的已发生成本。    
       function gT = gt(current_node, start,V)
            % 获取从起始节点到指定节点的路径代价
            %   current_node: 要计算其代价的节点 ,strat：起始点 V:节点集合
            
            % 如果节点是起始节点，则代价为 0。
            if isequal(current_node,start)
                gT = 0;
            % 如果节点不在树中，则代价为无穷大。
            elseif ~ismember(current_node,V)
                gT = Inf;
            % 返回通过树遍历从起始节点到指定节点的路径代价。
            else
                % par_cost 是从父节点到当前节点的代价，
                % parent.gt 是从起始节点到父节点的路径代价。
                gT = par_cost + parent.gt;
            end
       end
        
       
% g_(x) | 为X空间内点x已经发生的下界估计。   
        function g_hat = gen_g_hat(current_node,start)
            % current_node是当前节点，start是起始节点
            % 生成当前节点的 g_hat 值。这是当前节点与起始节点之间的 L2 范数。
            % 计算当前节点与起始节点之间的 L2 范数。
            g_hat = norm(current_node - start);
        end
        
 % h_(x) | 为X空间内点x的启发式的下界估计。h_(x,y):=c_(x,x_t)，其中x_t是通过规划问题找到的最佳解决方案的终点。
        function h_hat = gen_h_hat(current_node,goal)
            %current_node是当前节点，goal是目标节点             
            % 生成当前节点的 h_hat 值。这是当前节点与目标节点之间的 L2 范数。
            % 计算当前节点与目标节点之间的 L2 范数。
            h_hat = norm(current_node - goal);
        end
        
% 定义 11 (cost ← BestV alue(Qi))：查找 Qi 中队列成本最低的元素，并返回该元素的队列成本。      
        function a_hat_value = a_hat(start, node1, node2, goal)
         % start:起点，node1，第一个节点，node2：第二个节点，goal:Qi
         % 这是从起点到节点1的路径估计成本(L2范数) ，从节点1到节点2的路径估计成本(L2范数) ，以及从目标到节点2的启发式成本(L2范数)的总和。
         % 计算总估计成本，即 g_hat(node1) + c_hat(node1, node2) + h_hat(node2)
         a_hat_value = g_hat(start,node1) + c_hat(node1, node2) + h_hat(node2,goal);
        end        
    end
end
