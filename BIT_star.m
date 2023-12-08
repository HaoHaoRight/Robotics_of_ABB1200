classdef BIT_star
   %本代码由 https://arxiv.org/pdf/2302.11670.pdf 的伪代码部分改写而来

    properties
        Tree;% 树是一个有向无环图，表示为 T≡(V,E)
        Q;% 优先队列
        obstacle;
        x_root;% 根节点
        X_goal;% 机器人的目标位置
        X_flags;
        X_ncon;% 
        X_new;% 

    end
    
    methods
        function obj = BIT_star(start, goal, X_t, obstacle)
            % 构造函数，初始化算法
            % 输入：x_root-根节点
            %      X_goal-机器人的目标位置
            %      X_t-机器人的障碍物
            obj.x_root = start;
            obj.X_goal = goal;
            obj.obstacle = obstacle;

            %% 伪代码第一行
            Tree = struct('V',{},'E',{});% 树是一个有向无环图，表示为 T≡(V,E)
            % V是节点集合，E是边集合
            Tree.V = {x_r};% 初始树只有一个节点，即根节点
            Tree.V = {};

            %% 伪代码第二行
            Q = struct('V',{},'E',{});% 优先队列
            Q.V = Tree.V;
            Q.E = {};

            %% 伪代码第三行
            obj.X_flags = struct('X_new',{},'V_exp',{},'V_rewire',{},'V_sol',{},'c_sol',{});
            obj.X_ncon = X_goal;
            obj.X_flags.X_new = obj.X_ncon;
            

        end
        function ExpVertex(Tree, X_ncon, X_flags)
            
        end
        function randSample(obj, m)
            % 在X_free空间中随机采样。
            % X_free空间为所有不与障碍物相交的点的集合。
            % X_rand <- randSample(m)
            % 输入：m-采样点个数
        end
        function X_near = findNear(obj, x, X_search)
            % 在X_near空间中寻找与X_rand最近的点。
            % 找到X_serach内与点x欧几里得距离小于ρ的点。
            % X_near <- near(x, X_search)
            % 输入：x-点
            %      X_search-搜索空间
            X_near = {};
        end
    end
end