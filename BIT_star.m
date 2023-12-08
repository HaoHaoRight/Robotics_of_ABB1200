classdef BIT_star
   % 本代码由 https://arxiv.org/pdf/2302.11670.pdf 的伪代码部分改写而来
   % 英文部分主要取自原文，[]内为原文章节索引
   % 重要区别：x是点坐标，v是顶点(vetex)包含状态status和代价cost
   % 大写是集合，小写是元素
   %
   %% 【数据结构】
   % 坐标矩阵:x = [x;y;z]列向量
   % 顶点(vetex)结构体：v = struct('status',x,'cost',cost)，其中status是点坐标，cost是顶点代价
   % 边(edge)结构体：e = struct('father',v,'cost',cost)，其中father是父节点，cost是边代价
   % 为了保持代码简洁性，约定集合形如：
   % Tree.V = {
   %            struct('status',[],'cost',[]),
   %            struct('status',[],'cost',[]),
   %            ...
   %          } 
   % 每条边作为独立的实体，以Tree.V(index).Name的形式访问数据
   % 这可能会增大内存开销
    properties
        Tree;% 树是一个有向无环图(DAG)，表示为 T≡(V,E)
        Q;% 优先队列
        obstacle;
        x_root;% 根节点
        X_goal;% 机器人的目标位置
        X_flags;% X_new, V_exp, V_rewire, V_sol, c_sol
        X_ncon;% 尚未连接到树中的节点

        % [III] X_flags的定义
        % X_ncon ⊂ X | 是一个已被采样，但没有连接到搜索树的样本 集合。 is the set of all samples that are not connected to the search tree.
        % V_sol = V ∩ X_t | 包含已经到达目标区域的顶点，是V和X_t的 交集，。 is the set of vertices in V taht are also in target set X_t.
        % X_new ⊂ X_ncon | 包含最新的，还未评估是否加入搜索树的点，是最近一批样本的 集合。 is the set of samples that are from the most recent batch of samples.
        % c_sol = min{c(v), v ∈ V_sol} | 是到达目标区域的当前最小代价。 is the minimum cost of a path to the target set.
    end
    
    methods
        function obj = BIT_star(start, goal, X_t, obstacle)
            % Algorithm 1
            % 构造函数，初始化算法
            % 输入：x_root-根节点
            %      X_goal-机器人的目标位置
            %      X_t-机器人的障碍物
            obj.x_root = start;
            obj.X_goal = goal;
            obj.obstacle = obstacle;

            %% 伪代码第一行
            % 树是一个有向无环图，表示为 T≡(V,E)
            % V是节点集合，E是边集合
            % 树储存的节点是Node类的对象
            obj.Tree = {struct('V',[],'E',[])};
            obj.Tree.V = {struct('status',x_r,'cost',[])};% 初始树只有一个节点，即根节点
            obj.Tree.E = {struct('father',[],'cost',[])};% 边的结构体

            %% 伪代码第二行
            obj.Q = {struct('V',[],'E',[])};% 优先队列(queue)
            obj.Q.V = {struct('status',Tree.V,'cost',[])};% vertex queue
            obj.Q.E = {struct('father',[],'cost',[])};% edge queue

            %% 伪代码第三行
            obj.X_flags = {struct('X_new',[],'V_exp',[],'V_rewire',[],'V_sol',[],'c_sol',[])};
            obj.X_ncon = X_goal;% 尚未连接到树中的节点
            obj.X_flags.X_new = obj.X_ncon;
            obj.X_flags.c_sol = inf;
        end

        function [Q, X_flags] = ExpVertex(Tree, Q, X_ncon, X_flags)
            % [III.B] Algorithm 3
            % ExpVertex removes the lowest cost vertex from the queue Q.V and adds edges Q.E 
            % for every neighbor that might be part of the optimal path.
            % 输入：Tree-树
            %      Q-优先队列
            %      X_ncon-尚未连接到树中的节点
            %      X_flags- X_new, V_exp, V_rewire, V_sol, c_sol

            v_best = PopBest(Q, 'V');
            % Make edges to unconnected samples
            
            if ~ismember(v_best, X_flags.V_exp)
                X_flags.V_exp = {X_flags.V_exp, v_best};
                X_near = findNear(v_best, X_ncon);
            else % v_best has been expanded before
                X_near = findNear(v_best, intersect(X_flags.X_new, X_ncon));
            end

            % Q.E <-+ {(v_best, x), x ∈ X_near|g(v_best)+c(v_best,x)+h(x)<c_sol} if v_best not been rewired before
            % (v_best, x)表示一条从v_best到x的边  
            for i = 1:length(X_near)
                x = X_near(i);
                cost = g(v_best)+c(v_best,x)+h(x);%% 代写完整的cost函数
                if cost < X_flags.c_sol
                    Q.E = {Q.E, struct('father',v_best,'cost',cost)};% adds edges
                end
            end
        end

        function X_rand = randSample(obj, m)
            % [II.B Definiton 6]
            % 在X_free与椭球空间(informed set)的交集中随机采样。
            % X_free空间为所有不与障碍物相交的点的集合。
            % X_rand <- randSample(m)
            % 输入：m-采样点个数

            % 椭球的短轴是x_root到X_goal的距离
            c_min = dist(obj.x_root, obj.X_goal);
            % 椭球的长轴是当前最优路径长度c_best
            c_best = obj.X_flags.c_sol;
        end

        function X_near = findNear(obj, x, X_search)
            % [II.B Definiton 7]
            % 在X_near空间中寻找与X_rand最近的点。
            % 找到X_serach内与点x欧几里得距离小于ρ的点。
            % X_near <- near(x, X_search)
            % 输入：x-点
            %      X_search-搜索空间
            r = 0.1;
            X_near = [];
            distance = sqrt((x(1)-X_search(:,2)).^2+(x(2)-X_search(:,2)).^2+(x(3)-X_search(:,3)).^2);
            for i = 1:length(distance)
                if distance(i) < r
                    X_near = {X_near,X_search(i,:)};
                end
            end
        end

        function X_sol = Solution(obj, x)
            % [II.B Definiton 8]
            % Solution - 找到从根节点到节点 x 的路径。
            % 输入：x - 目标节点的坐标
            % 输出：X_sol - 从根节点到节点 x 的路径（节点坐标序列）
        
            % 初始化路径为一个空的单元数组
            X_sol = {};
        
            % 当前节点初始化为目标节点 x
            current_node = x;
        
            % 沿树回溯到根节点
            while ~isempty(current_node)
                % 将当前节点添加到路径数组的开始位置
                X_sol = [{current_node}, X_sol];
        
                % 如果当前节点是根节点，则停止回溯
                if isequal(current_node, obj.x_root)
                    break;
                end
        
                % 在 Tree.V 中找到 current_node 的索引
                node_index = find(arrayfun(@(v) isequal(v, current_node), obj.Tree.V));
                if isempty(node_index)
                    error('node not found in Tree.V');
                end
        
                % 使用 node_index 在 Tree.E 中找到对应的父节点
                father_node = obj.Tree.E(node_index).father;
        
                % 更新当前节点为父节点的坐标，继续回溯
                current_node = father_node;
            end
        end
        function x = PopBest(Q, name)
            % [II.B Definiton 12]
            % Finds the element with the lowest cost in the queue Q_i and removes it.
            % And returns the element.
            % PopBest - 从优先队列中弹出最优节点。
            % 输入：Q_i - 优先队列Q_E或Q_V
            % 输出：x - 最优节点('status', 'cost')
            if(name == 'V')
                [~, index] = min(Q.V.cost);
                x = Q.V(index);
                Q.V(index) = [];
            end

        end
        function distance = dist(x1, x2)
            % 计算两点间的欧几里得距离。
            % 输入：x1-点1
            %      x2-点2
            % 输出：distance-两点间的欧几里得距离
            distance = sqrt((x1(1)-x2(1))^2+(x1(2)-x2(2))^2+(x1(3)-x2(3))^2);
        end



    end
end