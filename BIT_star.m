classdef BIT_star
   % 本代码由 https://arxiv.org/pdf/2302.11670.pdf 的伪代码部分改写而来
   % 坐标矩阵:[x;
   %          y;
   %          z]
    properties
        Tree;% 树是一个有向无环图，表示为 T≡(V,E)
        Q;% 优先队列
        obstacle;
        x_root;% 根节点
        X_goal;% 机器人的目标位置
        X_flags;
        X_ncon;% 

    end
    
    methods
        function obj = BIT_star(start, goal, X_t, obstacle)
            % 构造函数，初始化算法
            % 输入：x_root-根节点
            %      X_goal-机器人的目标位置
            %      X_t-机器人的障碍物
            obj.x_root = Node(start, []);
            obj.X_goal = goal;
            obj.obstacle = obstacle;

            %% 伪代码第一行
            % 树是一个有向无环图，表示为 T≡(V,E)
            % V是节点集合，E是边集合
            % 树储存的节点是Node类的对象
            Tree = struct('V',{},'E',{});
            Tree.V = {x_r};% 初始树只有一个节点，即根节点
            Tree.V = {};
            Tree.E = struct('father',{},'child',{}, 'cost',{});% 边的结构体

            %% 伪代码第二行
            Q = struct('V',{},'E',{});% 优先队列
            Q.V = Tree.V;
            Q.E = struct('father',{},'child',{}, 'cost',{});% 边的结构体

            %% 伪代码第三行
            obj.X_flags = struct('X_new',{},'V_exp',{},'V_rewire',{},'V_sol',{},'c_sol',{});
            obj.X_ncon = X_goal;% 尚未连接到树中的节点
            obj.X_flags.X_new = obj.X_ncon;
            obj.X_flags.c_sol = inf;
        end
        function ExpVertex(Tree, X_ncon, X_flags)
            
        end
        function randSample(obj, m)
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
        function distance = dist(x1, x2)
            % 计算两点间的欧几里得距离。
            % 输入：x1-点1
            %      x2-点2
            % 输出：distance-两点间的欧几里得距离
            distance = sqrt((x1(1)-x2(1))^2+(x1(2)-x2(2))^2+(x1(3)-x2(3))^2);
        end
        function X_sol = Solution(obj, node)
            % Solution - 找到从根节点到节点 node 的路径。
            % 输入：node - 目标节点在树中的索引或标识符
            % 输出：X_sol - 从根节点到节点 node 的路径（节点序列）
            
            % 初始化路径为一个空的单元数组
            X_sol = {};
            
            % 当前节点初始化为目标节点 x
            current_node = node;
            
            % 沿树回溯到根节点
            while ~isempty(current_node)
                % 将当前节点添加到路径数组的开始位置
                X_sol = [{current_node}, X_sol];
                
                % 如果当前节点是根节点，则停止回溯
                if isequal(current_node, obj.x_root)
                    break;
                end
                
                % 获取当前节点的父节点
                parent_index = find(arrayfun(@(n) ismember(current_node, n.children), obj.Tree.V));
                if isempty(parent_index)
                    error('Parent node not found. The tree structure might be corrupted.');
                end
                
                % 更新当前节点为父节点，继续回溯
                current_node = obj.Tree.V(parent_index).node;
            end
        end
    end
end