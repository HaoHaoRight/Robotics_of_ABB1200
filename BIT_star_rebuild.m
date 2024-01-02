classdef BIT_star_rebuild
    % 定义所有向量为行向量(i,:)
    % Tree = (V, E)
    % Q = (V, E)
    % V = {v}
    % E = {(v,x)}
    % 长度用size()
    properties
        Tree;
        Q;
        V_old;
        cost;
        obstacle;
        x_root;
        x_goal;
        X_samples;% The set of unconnected samples
        demension;
        radius;
        m;
    end % end properties
    methods
        function obj = BIT_star_rebuild(obstacle, x_root, x_goal, demension, m)
            obj.obstacle = obstacle;
            obj.x_root = x_root;
            obj.x_goal = x_goal;
            obj.cost = costs(obj.x_root, obj.x_goal, obj.obstacle);
            obj.demension = demension;
            obj.Tree = struct('V',[],'E',[]);
            obj.Tree.E = struct('v',[],'x',[]);
            obj.Tree.V = [obj.x_root];
            obj.Q = struct('V',[],'E',[]);
            obj.Q.V = [];
            obj.Q.E = struct('v',[],'x',[]);
            obj.X_samples = [obj.x_goal];
            obj.V_old = [];
            obj.m = m;
        end
        function path = Solution(obj)
            if isempty(obj.Q.V) && isempty(obj.Q.E.v) && isempty(obj.Q.E.x)
                % Batch creation
                Prune(obj, obj.cost.gT(obj.x_goal, obj.Tree));
                % Prune(gT(x_goal))
                obj.X_samples = [obj.X_samples; obj.Sample(obj.m, obj.cost.gT(obj.x_goal, obj.Tree))];
                % Sample(m, gT(x_goal))
                obj.V_old = obj.Tree.V;
                obj.Q.V = obj.Tree.V;
                obj.radius = 10;
                path = obj.Tree;
            end
            while obj.cost.BestValue(obj.Q,'V',obj.Tree) <= obj.cost.BestValue(obj.Q,'E',obj.Tree)

            end
        end

        function Prune(obj, c)
            % Prune the tree (g_(x)+h_(x) > c)
            for i = 1:size(obj.X_samples)
                if obj.cost.g_(obj.X_samples(i,:))+obj.cost.h_(obj.X_samples(i,:)) > c
                    obj.X_samples(i,:) = [];
                end
            end
            for i = 1:size(obj.Tree.V)
                if obj.cost.g_(obj.Tree.V(i,:))+obj.cost.h_(obj.Tree.V(i,:)) > c
                    obj.Tree.V(i,:) = [];
                end
            end
            for i = 1:size(obj.Tree.E.v)
                if obj.cost.g_(obj.Tree.E.v(i,:))+obj.cost.h_(obj.Tree.E.v(i,:)) > c
                    obj.Tree.E.v(i,:) = [];
                    obj.Tree.E.x(i,:) = [];
                end
            end
            for i = 1:size(obj.V_old)
                if obj.cost.g_(obj.V_old(i,:))+obj.cost.h_(obj.V_old(i,:)) > c
                    obj.V_old(i,:) = [];
                end
            end
            
        end

        function X_samples = Sample(obj, m, c) 
            % 拒绝采样版本
            X_samples = zeros(m, obj.demension);
            count = 1;
            while count <= m
                rand_point = rand(1,obj.demension); % 生成随机点（n维行向量）
                if norm(rand_point-obj.x_root) + norm(rand_point-obj.x_goal) <= c && ~obj.obstacle.isVectorIntersectingObstacle(rand_point, obj.x_goal)
                    X_samples(count, :) = rand_point; % 添加行向量到矩阵
                    count = count + 1;
                end
            end
        end
        
        function ExpandVertex(obj)
            % [Alg.2]
            % Pop the best vertex from Q.V
            [~, index] = obj.cost.BestValue(obj.Q,'V',obj.Tree);
            v = obj.Q.V(index,:);
            obj.Q.V(index,:) = [];

            X_near = obj.Near(v, obj.X_samples);

            % QE ←+ (v, x) ∈ (V x X_near) 
            for i = 1:size(X_near)
                x = X_near(i,:);
                for j = 1:size(obj.Tree.V)
                    v = obj.Tree.V(j,:);
                    if obj.cost.g_(v)+obj.cost.c_(v,x)+obj.cost.h_(x) < obj.cost.gT(obj.x_goal)
                        obj.Q.E.v = [obj.Q.E.v; v];
                        obj.Q.E.x = [obj.Q.E.x; x];
                    end
                end
            end

            if  ~ismember(v, obj.V_old, 'rows')
                V_near = obj.Near(v, obj.Tree.V);
                % QE ←+ (v, w) ∈ (V x V_near) 
                for i = 1:size(V_near)
                    w = V_near(i,:);
                    for j = 1:size(obj.Tree.V)
                        v = obj.Tree.V(j,:);
                        if ~any(obj.Q.E.x(obj.Q.E.v == v) == w)
                        % (v, w) ∉ E
                            if obj.cost.g_(v)+obj.cost.c_(v,w)+obj.cost.h_(w) < obj.cost.gT(obj.x_goal) && obj.cost.gT(v, obj.Tree) + obj.cost.c_(v,w) < obj.cost.gT(w, obj.Tree)
                                obj.Q.E.v = [obj.Q.E.v; v];
                                obj.Q.E.x = [obj.Q.E.x; w];
                            end
                        end
                    end
                end
            end
        end

        function X_near = Near(obj, v, Samples)
            X_near = [];
            for i = 1:size(Samples)
                if norm(Samples(i,:)-v) <= obj.radius
                    X_near = [X_near; Samples(i,:)];
                end
            end
        end
    end % end methods
end % end classdef