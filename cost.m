classdef cost
    % [II.B Definition 1-5, 11] 成本函数
    %% TODO
    % c(x,y) | 计算X空间内点x到y的路径长度。如果阻塞则返回无穷大，或者路径不存在。
    % c_(x,y) | 计算X空间内点y∈X_i的路径成本的下界估计。c_不考虑障碍物。使用欧几里得距离定义为 c_(x,y):=∥x−y∥。
    % gT(x) | 计算根节点通过树Tree到X空间内点x的已发生成本。
    % g_(x) | 为X空间内点x已经发生的下界估计。
    % h_(x) | 为X空间内点x的启发式的下界估计。h_(x,y):=c_(x,x_t)，其中x_t是通过规划问题找到的最佳解决方案的终点。
    % BestValue(obj, Q) | 返回Q中最佳节点的成本。

    properties

    end

    methods
        function BestValue(obj, Q)

        end
    end
end
