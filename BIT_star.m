classdef BIT_star
   % 本代码由 https://arxiv.org/pdf/2302.11670.pdf 的伪代码部分改写而来
   % 英文部分主要取自原文，[]内为原文章节索引
   % 重要区别：x是点坐标，v是顶点(vertex)
   % 大写是集合，小写是元素
   
   %% 【数据结构】
   % 坐标矩阵:x = [x;y;z]列向量
   % 边(edge)结构体：e = struct('father',v,'cost',cost)，其中father是父节点，cost是边代价
   % 为了保持代码简洁性，约定集合形如：
   % Tree.E = {
   %            struct('father',[],'cost',[]),
   %            struct('father',[],'cost',[]),
   %            ...
   %          } 
   % 每条边作为独立的实体，以Tree.V(index).Name的形式访问数据
   % 这可能会增大内存开销

   % 我还没有完全理解重连操作rewire，待理解
   % 为避免混肴，把V定义为matrix，E定义为cell
   % 这样可以使x和v同源，而v则是树中的顶点
   % 因为V.cost多余了，点的成本完全可以单独计算
   % 不确定isempty(Q.V)是否正确

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
        % V_re
        % c_sol = min{c(v), v ∈ V_sol} | 是到达目标区域的当前最小代价。 is the minimum cost of a path to the target set.
    end
    
    methods
        function X_sol = BIT_star(start, goal, X_t, obstacle)
            % Algorithm 1
            % 构造函数，初始化算法
            % 输入：x_root-根节点
            %      X_goal-机器人的目标位置
            %      X_t-机器人的目标区域
            obj.x_root = start;
            obj.X_goal = goal;
            obj.obstacle = obstacle;

            % Line 1
            % 树是一个有向无环图，表示为 T≡(V,E)
            % V是节点集合，E是边集合
            % 树储存的节点是Node类的对象
            obj.Tree = {struct('V',[],'E',[])};
            obj.Tree.V = {x_root};% 初始树只有一个节点，即根节点
            obj.Tree.E = {struct('father',[],'cost',[])};% 边的结构体

            %Line 2
            obj.Q = {struct('V',[],'E',[])};% 优先队列(queue)
            obj.Q.V = {Tree.V};% vertex queue
            obj.Q.E = {struct('father',[],'cost',[])};% edge queue

            % Line 3-6
            obj.X_flags = {struct('X_new',{},'V_exp',{},'V_rewire',{},'V_sol',{},'c_sol',{})};
            obj.X_ncon = X_goal;% 尚未连接到树中的节点
            obj.X_flags.X_new = obj.X_ncon;
            obj.X_flags.c_sol = inf;
            obj.X_flags.V_sol = obj.X_goal;

            % Line 5
            if isempty(obj.X_flags.V_sol)
                obj.X_flags.c_sol = inf;
            else
                % c_sol <- minv∈V_sol gT(v) 待优化
                gTV = {};
                for i = 1:length(obj.X_flags.V_sol)
                    gTV = {gTV, gT(obj.X_flags.V_sol)};%gT待实现
                end
                obj.X_flags.c_sol = min(gTV);%gT待实现
            end

            % Line 7-17
            while stop
                % Line 8
                % Prune sub-optimal nodes
                if xor(isempty(Q.V), isempty(Q.E)) % End of batch
                    

                % Line 9
                % Process best vertex available
                elseif BestVertex(Q, 'V') <= BestVertex(Q, 'E') 
                    Q, X_flags = ExpVertex(Tree, Q, X_ncon, X_flags);

                %Line 15
                % Process best edge available
                else
                    Tree, Q, X_ncon, X_flags = ExpEdge(Tree, Q, X_ncon, X_flags);% 待实现
                end
            end
        end
        function [X_reuse, Tree, X_ncon, X_flags] = Prune(Tree, X_ncon, X_flags)
            % [III.B] Algorithm 2
            % 输入：Tree-树 T≡(V,E)
            %      X_ncon-尚未连接到树中的节点
            %      X_flags- X_flags≡(X_new,V_exp,V_rewire,V_sol,c_sol)
            % Line 1
            X_reuse = {};
            % Line 2
            % Removes all umconnected samples with cost.g_(X_ncon(i)) + cost.h_(X_ncon(i)) >= X_flags.c_sol
            % Note that this can be thought of as removeing all nodes that fall outside of the "informed set" and as such provably cannot contribute to the optimal solution.
            % 这可以理解为从“启发集合”中移除那些点。启发集合是 Informed RRT* 算法中定义的一个概念，它包括那些仍有可能贡献于最佳解决方案的节点。
            for i = 1:length(X_ncon)
                if cost.g_(X_ncon(i)) + cost.h_(X_ncon(i)) >= X_flags.c_sol
                    X_ncon(i) = [];
                end
            end
            % 去除X_ncon中的空元素
            X_ncon = X_ncon(~cellfun('isempty',X_ncon));
            
            % Line 3-7
            for i = 1:length(Tree.V)
                if cost.g_(Tree.V(i)) + cost.h_(Tree.V(i)) > X_flags.c_sol
                    Tree.V(i) = [];
                    Tree.E(i) = [];
                    X.flags.V_rewire(i) = [];
                    X.flags.V_exp(i) = [];
                    X_t(i) = [];
                    % If v can possibility help
                    if cost.g_(Tree.V(i)) + cost.h_(Tree.V(i)) < X_flags.c_sol
                        X_reuse = {X_reuse, Tree.V(i)};
                    end
                end
                % 去除空元素
                Tree.V = Tree.V(~cellfun('isempty',Tree.V));
                Tree.E = Tree.E(~cellfun('isempty',Tree.E));
                X.flags.V_rewire = X.flags.V_rewire(~cellfun('isempty',X.flags.V_rewire));
                X.flags.V_exp = X.flags.V_exp(~cellfun('isempty',X.flags.V_exp));
                X_t = X_t(~cellfun('isempty',X_t));
            end
        end
        function [Q, X_flags] = ExpVertex(Tree, Q, X_ncon, X_flags)
            % [III.B] Algorithm 3
            % ExpVertex removes the lowest cost vertex from the queue Q.V and adds edges Q.E 
            % for every neighbor that might be part of the optimal path.
            % Note that ExpVertex is without obstacel checking. 
            % 输入：Tree-树 T≡(V,E)
            %      Q-优先队列 Q≡(Q.V,Q.E)
            %      X_ncon-尚未连接到树中的节点
            %      X_flags- X_flags≡(X_new,V_exp,V_rewire,V_sol,c_sol)

            % Line 1
            v_best = PopBest(Q, 'V');
            % Make edges to unconnected samples

            % Line 2-6
            if ~ismember(v_best, X_flags.V_exp)
                X_flags.V_exp = {X_flags.V_exp, v_best};
                X_near = findNear(v_best, X_ncon);
            else % v_best has been expanded before
                X_near = findNear(v_best, intersect(X_flags.X_new, X_ncon));
            end

            % Line 7
            % Q.E <-+ {(v_best, x), x ∈ X_near|g_(v_best)+c_(v_best,x)+h_(x)<c_sol} if v_best not been rewired before
            % (v_best, x)表示一条从v_best到x的边  
            for i = 1:length(X_near)
                x = X_near(i);
                cost = cost.g_(v_best)+cost.c_(v_best,x)+cost.h_(x);%% 代写完整的cost函数
                if cost < X_flags.c_sol
                    Q.E = {Q.E, struct('father',v_best,'cost',cost)};% adds edges
                end
            end

            % Line 8-10
            if ~ismember(v_best, X_flags.V_rewire) ^ X_flags.V_rewire.c_sol < inf
                % v_best是否已经不在重连顶点集合 XOR 至少找到了一个解
                % 以下条件仅一个为真时成立：1.v_best之前没有考虑过重连，第一个解尚未找到 2.v_best之前考虑过重连，但是此时已经有一个解了
                % 作用：在还没有找到任何解的情况下，算法会跳过重连操作，以更快地找到第一个解。一旦找到第一个解，算法则会考虑之前跳过的重连操作，以进一步优化和改进找到的解。
                % When v_best might be a better parent for its neighbors than their current parent.
                % Skip any potential rewirings.

                % Line 9
                X_flags.V_rewire = {X_flags.V_rewire, v_best};
                X_search = cell2mat(Tree.V); % X_search is a matrix

                % Line 10
                X_flags.V_near = findNear(v_best, X_search);% Finds all vertes in the serach tree that are near v_best

                % Line 11
                % Adds all edges (v_best, V_near) to the queue Q.E that are not already part of the tree and might be part of the optimal path.
                % Q.E <-+ {(v_best, w), w ∈ V_near|(v_best,w)∈E and g_(v_best)+c_(v_best,w)<gT(w),g_(v_best)+c_(v_best,w)+h_(w)<c_sol}
                for i = 1:length(X_flags.V_near)
                    w = X_flags.V_near(i);
                    % 代价函数待实现
                    if ismember(struct('father',v_best,'cost',[]), Tree.E) && cost.g_(v_best)+cost.c_(v_best,w)<cost.gT(w) && cost.g_(v_best)+cost.c(v_best,w)+cost.h_(w)<X_flags.c_sol
                        Q.E = {Q.E, struct('father',v_best,'cost',g(v_best)+c(v_best,w))};
                    end
                end
            end
        end
        function [Tree, Q, X_ncon, X_flags] = ExpEdge(Tree, Q, X_ncon, X_flags)
            % [III.B] Algorithm 4
            % ExpEdge removes the lowest cost edge from the queue Q.E and adds the vertex to the tree.
            % Note that ExpEdge is without obstacle checking.
            % 输入：Tree-树 T≡(V,E)
            %      Q-优先队列 Q≡(Q.V,Q.E)
            %      X_ncon-尚未连接到树中的节点
            %      X_flags- X_flags≡(X_new,V_exp,V_rewire,V_sol,c_sol)

            % Line 1
            [v_best, x_best] = PopBest(Q, 'E');

            % Line 2-4
            % The best edge in the edge queue cannot improve the solution
            if cost.g_(v_best)+cost.c_(v_best, x_best)+cost.h_(x_best)
                Q.E = [];
                Q.V = [];
                return;
            end

            % Line 5-10
            % x_best is unconnected
            if ~ismember(x_best, X_ncon)
                % Adds x_best to the tree
                if cost.g_(v_bst)+cost.c_(v_best, x_best)+cost.h_(x_best) < X_flags.c_sol
                    % X_ncon中去除x_best
                    X_ncon = X_ncon(~ismember(X_ncon, x_best));
                    % Tree.V <-+ x_best
                    Tree.V = {Tree.V, x_best};
                    % Tree.E <-+ (v_best, x_best)
                    Tree.E = {Tree.E, struct('father',v_best,'cost',cost.g_(v_best)+cost.c_(v_best, x_best))};
                    % Q.V <-+ x_best
                    Q.V = {Q.V, x_best};
                    if ismember(x_best, X_t)% 实际实现不用集合论
                        % X_flags.V_sol <-+ x_best
                        X_flags.V_sol = {X_flags.V_sol, x_best};
                        % X_flags.c_sol <-+ minv_sol∈V_sol gT(v_sol)
                        for i = 1:length(X_flags.V_sol)
                            gTV_sol = {gTV_sol, gT(X_flags.V_sol(i))};
                        end
                        X_flags.c_sol = min(gTV_sol);
                    end
                end

            % Line 11
            % x_best is connected
            else
                % Line 12
                if cost.gT(v_best)+cost.c_(v_best, x_best)<cost.gT(x_best)
                    % Line 13
                    if cost.gT(v_best)+cost.c(v_best, x_best)+cost.h_(x_best)<X_flags.c_sol
                        % Line 14
                        if cost.gT(v_best)+cost.c(v_best, x_best)<cost.gT(x_best)
                            % Line 15
                            % Tree.E <-- (Par(x_best), x_best)
                            % Tree.E <-- (v_best, x_best)
                            % 找到x_best在Tree.V中的索引
                            index = find(cellfun(@(x) isequal(x, x_best), Tree.V));
                            Tree.E(index) = struct('father',v_best,'cost',cost.gT(v_best)+cost.c(v_best, x_best));

                            % Line 16
                            % c_sol <- minv_sol∈V_sol gT(v_sol)
                            for i = 1:length(X_flags.V_sol)
                                gTV_sol = {gTV_sol, gT(X_flags.V_sol(i))};
                            end
                            X_flags.c_sol = min(gTV_sol);
                        end
                    end
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
            c_min = obj.dist(obj.x_root, obj.X_goal);
            % 椭球的长轴是当前最优路径长度c_best
            c_best = obj.X_flags.c_sol;
        end

        function X_near = findNear(obj, v, X_search)
            % [II.B Definiton 7]
            % 在X_near空间中寻找与X_rand最近的点。
            % 找到X_serach内与点x欧几里得距离小于ρ的点。
            % X_near <- near(x, X_search)
            % 输入：v-顶点
            %      X_search-搜索空间
            r = 0.1;
            X_near = [];
            x = v;
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
            % 输出：x - 最优节点
            if(name == 'V')
                [~, index] = min(cost.g_(Q.V)+cost.h_(Q.V));% 待确认
                x = Q.V(index);
                Q.V(index) = [];
                Q.V = Q.V(~cellfun('isempty',Q.V));% 去除空元素
            end
            if(name == 'E')
                [~, index] = min(Q.E(:).cost);
                x = [Q.E(index).father, Q.V(index)];
                Q.E(index) = [];
                Q.E = Q.E(~cellfun('isempty',Q.E));% 去除空元素
            end
        end
        function distance = dist(x1, x2)
            % 计算两点间的欧几里得距离。
            % 输入：x1-点1
            %      x2-点2
            % 输出：distance-两点间的欧几里得距离
            distance = sqrt((x1(1)-x2(1))^2+(x1(2)-x2(2))^2+(x1(3)-x2(3))^2);
        end

    end % methods end
end % classdef end