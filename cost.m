classdef cost
    % [II.B Definition 1-5] 成本函数
    % c(x,y) | 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。
    % c_(x,y) | 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
    % gT(x) | 计算根节点通过树Tree到X空间内点x的已发生成本。
    % g_(x) | 为X空间内点x已经发生的下界估计。
    % h_(x) | 为X空间内点x的启发式的下界估计。h_(x,y):=c_(x,x_t)，其中x_t是通过规划问题找到的最佳解决方案的终点。
    properties
    tree;%树
    x_root;%根节点
    X_goal;%目标点
    end

    methods
       function obj = Cost(tree)
            % 构造函数
            obj.tree = tree;  % 假设树作为参数输入
            obj.x_root=start;
            obj.X_goal = goal;
            obj.V = nodes;
       end
            function h_x = h_x(obj, x_t)
               % 计算点 x 到 x_t 的路径成本 c_(x, x_t)
                  cost_x_to_xt = obj.c(x, x_t);  % 请确保您已经实现了 c 函数来计算点 x 到 x_t 的路径成本

                  % 使用 c_(x, x_t) 作为 h_(x) 的下界估计
                h_x = cost_x_to_xt;
             end

        function gT = gt(obj, node)
            % 获取从起始节点到指定节点的路径代价
            % Args:
            %   node: 要计算其代价的节点
            
            % 如果节点是起始节点，则代价为 0。
            if isequal(node, obj.start)
                gT = 0;
            % 如果节点不在树中，则代价为无穷大。
            elseif ~ismember(node, obj.V)
                gT = Inf;
            % 返回通过树遍历从起始节点到指定节点的路径代价。
            else
                % node.par_cost 是从父节点到当前节点的代价，
                % node.parent.gt 是从起始节点到父节点的路径代价。
                gT = node.par_cost + node.parent.gt;
            end
       
       function gTlowCost = gt(node)
                gTlowCost=node.gen_g_hat();
       end
                
       function pathLength = c( x, y, obstacle)
            % 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。
            if obstacle.isVectorIntersectingObstacle(x, y)
                pathLength = Inf; % 如果相交，返回无穷大
                return;
            end
            pathLength = norm(x - y); % 计算路径长度
       end
       function pathLengthlowCost = clow(x, y)
            % 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。
            % 使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
            pathLengthlowCost = norm(x - y); % 使用欧几里得距离作为路径成本的下界估计
       end

        
        function BestValue(obj, Q)

        end
    end
end
