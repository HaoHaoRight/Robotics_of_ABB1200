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
        cost_old;
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
            obj.Q.E = struct('v',[],'x',[],'cost',[]);
            obj.X_samples = [obj.x_goal];
            obj.V_old = [];
            obj.m = m;
            obj.cost_old = inf;
        end
        function path = Solution(obj)
            batch_count = 1;
            while 1
                if isempty(obj.Q.V) && isempty(obj.Q.E.v) && isempty(obj.Q.E.x)
                    % Batch creation
                    obj = Prune(obj, obj.cost.gT(obj.x_goal, obj.Tree));
                    % Prune(gT(x_goal))
                    obj.X_samples = [obj.X_samples; obj.Sample(obj.m, obj.cost.gT(obj.x_goal, obj.Tree))];
                    obj.Plot(batch_count);
                    batch_count = batch_count + 1;
                    % Sample(m, gT(x_goal))
                    obj.V_old = obj.Tree.V;
                    obj.Q.V = obj.Tree.V;
                    obj.radius = 0.08;
                end
                [QV_BestValue, QV_BestIndex] = obj.cost.BestValue(obj.Q,'V',obj.Tree);
                [QE_BestValue, QE_BestIndex] = obj.cost.BestValue(obj.Q,'E',obj.Tree);
                while QV_BestValue <= QE_BestValue && ~isempty(obj.Q.V)
                    obj = obj.ExpandVertex(QV_BestIndex);
                    QV_BestValue = obj.cost.BestValue(obj.Q,'V',obj.Tree);
                end
                if ~isempty(obj.Q.E.v)
                    obj = obj.ExpandEdge(QE_BestIndex);
                end

                cost_new = obj.cost.gT(obj.x_goal, obj.Tree);
                if cost_new  < obj.cost_old
                    fprintf('Now cost = %f\n', cost_new);
                    percent_change = abs((obj.cost_old - cost_new) / obj.cost_old)*100;
                    fprintf('Cost Change Rate = %f %%\n', percent_change);
                    if cost_new < 0.525
                    %if percent_change < 0.03  % 阈值
                        path = obj.Path();
                        break
                    end
                    obj.cost_old = cost_new;
                end
            end

        end

        function obj = Prune(obj, c)
            % Prune the tree (g_(x)+h_(x) > c)
            % 定义删除索引的数组
            del_idx_X_samples = false(size(obj.X_samples, 1), 1);
            del_idx_Tree_V = false(size(obj.Tree.V, 1), 1);
            del_idx_Tree_E = false(size(obj.Tree.E.v, 1), 1);
            del_idx_V_old = false(size(obj.V_old, 1), 1);
            for i = size(obj.X_samples):-1:1
                if norm(obj.X_samples(i,:)-obj.x_root)+norm(obj.X_samples(i,:)-obj.x_goal) > c
                    del_idx_X_samples(i) = true;
                end
            end
            obj.X_samples(del_idx_X_samples,:) = [];
            
            for i = size(obj.Tree.V):-1:1
                if obj.cost.g_(obj.Tree.V(i,:))+obj.cost.h_(obj.Tree.V(i,:)) > c
                    del_idx_Tree_V(i) = true;
                end
            end
            obj.Tree.V(del_idx_Tree_V,:) = [];

            for i = size(obj.Tree.E.v):-1:1
                if obj.cost.g_(obj.Tree.E.v(i,:))+obj.cost.h_(obj.Tree.E.v(i,:)) > c || obj.cost.g_(obj.Tree.E.x(i,:))+obj.cost.h_(obj.Tree.E.x(i,:)) > c
                    del_idx_Tree_E(i) = true;
                end
            end
            obj.Tree.E.v(del_idx_Tree_E,:) = [];
            obj.Tree.E.x(del_idx_Tree_E,:) = [];

            for i = size(obj.V_old):-1:1
                if obj.cost.g_(obj.V_old(i,:))+obj.cost.h_(obj.V_old(i,:)) > c
                    del_idx_V_old(i) = true;
                end
            end
            obj.V_old(del_idx_V_old,:) = [];

            del_idx_Tree_V = false(size(obj.Tree.V, 1), 1);
            for i = size(obj.Tree.V):-1:1
                if obj.cost.gT(obj.Tree.V(i,:), obj.Tree) == inf
                    obj.X_samples = [obj.X_samples; obj.Tree.V(i,:)];
                    del_idx_Tree_V(i) = true;
                end
            end
            obj.Tree.V(del_idx_Tree_V,:) = [];
        end

        function Samples = Sample(obj, m, c)
            % 拒绝采样版本
            Samples = zeros(m, obj.demension);
            dem = obj.demension;
            root = obj.x_root;
            goal = obj.x_goal;
            obs = obj.obstacle;
            % 使用 parfor 生成样本
            parfor i = 1:m
                valid_sample = false;
                while ~valid_sample
                    rand_point = rand(1, dem); % 生成随机点（n维行向量）
                    if norm(rand_point-root) + norm(rand_point-goal) <= c && ~obs.isPointInside(rand_point)
                        Samples(i, :) = rand_point; % 添加行向量到矩阵
                        valid_sample = true;
                    end
                end
            end
        end
        
        function obj = ExpandVertex(obj, QV_BestIndex)
            % [Alg.2]
            % Pop the best vertex from Q.V
            index = QV_BestIndex;
            v = obj.Q.V(index,:);
            obj.Q.V(index,:) = [];

            X_near = obj.Near(v, obj.X_samples);

            % QE ←+ (v, x) ∈ (V x X_near) 
            for i = 1:size(X_near)
                x = X_near(i,:);
                if obj.cost.g_(v)+obj.cost.c_(v,x)+obj.cost.h_(x) < obj.cost.gT(obj.x_goal, obj.Tree)
                    obj.Q.E.v = [obj.Q.E.v; v];
                    obj.Q.E.x = [obj.Q.E.x; x];
                end
            end

            if  ~ismember(v, obj.V_old, 'rows')
                V_near = obj.Near(v, obj.Tree.V);
                % QE ←+ (v, w) ∈ (V x V_near) 
                for i = 1:size(V_near)
                    w = V_near(i,:);
                    if ~ismember(w, obj.Q.E.x(ismember(obj.Q.E.v, v, 'rows'), :), 'rows')
                        % (v, w) ∉ E
                        if obj.cost.g_(v)+obj.cost.c_(v,w)+obj.cost.h_(w) < obj.cost.gT(obj.x_goal,obj.Tree) && obj.cost.gT(v,obj.Tree) + obj.cost.c_(v,w) < obj.cost.gT(w, obj.Tree)
                            obj.Q.E.v = [obj.Q.E.v; v];
                            obj.Q.E.x = [obj.Q.E.x; w];
                        end
                    end
                end
            end
        end

        function obj = ExpandEdge(obj, QE_BestIndex)
            % [Alg.2]
            % Pop the best edge from Q.E
            index = QE_BestIndex;
            v = obj.Q.E.v(index,:);
            x = obj.Q.E.x(index,:);
            obj.Q.E.v(index,:) = [];
            obj.Q.E.x(index,:) = [];

            if obj.cost.gT(v,obj.Tree)+obj.cost.c_(v,x)+obj.cost.h_(x) < obj.cost.gT(obj.x_goal,obj.Tree)
                if obj.cost.g_(v)+obj.cost.c_(v,x)+obj.cost.h_(x) < obj.cost.gT(obj.x_goal,obj.Tree)
                    if obj.cost.gT(v,obj.Tree) + obj.cost.c(v,x) < obj.cost.gT(x,obj.Tree)
                        if ismember(x, obj.Tree.V, 'rows')
                            % x ∈ V
                            obj.Tree.E.v = obj.Tree.E.v(~ismember(obj.Tree.E.x, x, 'rows'),:);
                            obj.Tree.E.x = obj.Tree.E.x(~ismember(obj.Tree.E.x, x, 'rows'),:);
                        else
                            % x ∉ V
                            obj.X_samples = obj.X_samples(~ismember(obj.X_samples, x, 'rows'),:);
                            obj.Tree.V = [obj.Tree.V; x];
                            obj.Q.V = [obj.Q.V; x];
                        end
                        obj.Tree.E.v = [obj.Tree.E.v; v];
                        obj.Tree.E.x = [obj.Tree.E.x; x];

                        rows_to_remove = false(size(obj.Q.E.v, 1), 1);  % 逻辑索引数组

                        for i = size(obj.Q.E.v, 1):-1:1
                            vi = obj.Q.E.v(i,:);
                            if obj.cost.gT(vi, obj.Tree) + obj.cost.c_(vi, x) >= obj.cost.gT(x, obj.Tree)
                                rows_to_remove(i) = true;
                            end
                        end

                        % 删除标记的行
                        obj.Q.E.v(rows_to_remove, :) = [];
                        obj.Q.E.x(rows_to_remove, :) = [];
                    end
                end
            else
                obj.Q.E.v = [];
                obj.Q.E.x = [];
                obj.Q.V = [];
                % start new batch
            end
        end

        function X_near = Near(obj, x, Samples)
            % X_near ←+ x′ ∈ X : ||x′ − x|| ≤ r
            X_near = [];
            for i = 1:size(Samples)
                if (norm(Samples(i,:)-x) <= obj.radius) && ~isequal(Samples(i,:),x)
                    X_near = [X_near; Samples(i,:)];
                end
            end
        end

        function path = Path(obj)
            path = [];
            current = obj.x_goal;
            while ~isempty(obj.x_root)
                path = [current; path];
                if isequal(obj.x_root, current)
                    break
                end
                current = obj.Tree.E.v(ismember(obj.Tree.E.x, current, 'rows'),:);
            end
        end
        function Plot(obj, batch_count)
            figure;
            hold on;
            axis equal;
            grid on;
            plot3(obj.X_samples(:,1), obj.X_samples(:,2), obj.X_samples(:,3), 'b.');
            title('Batch',batch_count);
            view(80, 30);
            hold off;
        end  
    end % end methods
end % end classdef