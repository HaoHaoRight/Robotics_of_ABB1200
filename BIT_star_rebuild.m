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
        volume_Xf;
    end % end properties
    methods
        function obj = BIT_star_rebuild(obstacle, x_root, x_goal, demension, m)
            obj.obstacle = obstacle;
            obj.x_root = x_root;
            obj.x_goal = x_goal;
            obj.cost = costs(obj.x_root, obj.x_goal, obj.obstacle);
            obj.demension = demension;

            obj.Tree = struct('V',[],'E',[]);
            obj.Tree.E = struct('v',[],'x',[],'father_i',[]);
            obj.Tree.V = [obj.x_root];

            obj.Q = struct('V',[],'E',[]);
            obj.Q.V = [];
            obj.Q.E = struct('v',[],'x',[]);

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
                    [Sample, obj.volume_Xf] = obj.Sample(obj.m, obj.cost.gT(obj.x_goal, obj.Tree));
                    obj.X_samples = [obj.X_samples; Sample];
                    % obj.Plot(batch_count);
                    batch_count = batch_count + 1;
                    % Sample(m, gT(x_goal))
                    obj.V_old = obj.Tree.V;
                    obj.Q.V = obj.Tree.V;
                    obj.radius = obj.Radius(size(obj.X_samples), 1.2);
                end
                while obj.cost.BestValue(obj.Q,'V',obj.Tree) <= obj.cost.BestValue(obj.Q,'E',obj.Tree) && ~isempty(obj.Q.V)
                    obj = obj.ExpandVertex();
                end
                if ~isempty(obj.Q.E.v)
                    obj = obj.ExpandEdge();
                end
                
                % obj.cost.Cache = dictionary(obj.x_root,0);
                cost_new = obj.cost.gT(obj.x_goal, obj.Tree);
                if cost_new  < obj.cost_old
                    
                    %obj = obj.Path_Simplification();
                    %obj.cost.Cache = dictionary(obj.x_root,0);% 清空字典
                    
                    fprintf('Now cost = %f\n', cost_new);

                    %percent_change = abs((obj.cost_old - cost_new) / obj.cost_old)*100;
                    %fprintf('Cost Change Rate = %f %%\n', percent_change);
                    if batch_count >= 1000 ||cost_new < 0.75 % 终止条件
                    %if percent_change < 0.03  % 阈值
                        path = obj.Path();
                        fprintf('cost before simplification = %f\n', obj.calculatePathLength(path));
                        
                        path = obj.douglasPeucker(path, 0.01);
                        fprintf('cost after simplification = %f\n', obj.calculatePathLength(path));
                        break
                    end
                    obj.cost_old = cost_new;
                end
            end
        end
        
        function obj = Prune(obj, c)
            % Prune the tree (g_(x)+h_(x) > c)
            % 定义删除索引的数组
            obj.cost.Cache = dictionary(obj.x_root,0);% 清空字典
            obj = obj.Path_Simplification();
            %obj.cost.Cache = dictionary(obj.x_root,0);% 清空字典
            
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
            obj.cost.isTreeSame(obj.Tree);
            
            for i = size(obj.Tree.E.v):-1:1
                if obj.cost.g_(obj.Tree.E.v(i,:))+obj.cost.h_(obj.Tree.E.v(i,:)) > c || obj.cost.g_(obj.Tree.E.x(i,:))+obj.cost.h_(obj.Tree.E.x(i,:)) > c
                    del_idx_Tree_E(i) = true;
                end
            end
            obj.Tree.E.v(del_idx_Tree_E,:) = [];
            obj.Tree.E.x(del_idx_Tree_E,:) = [];
            obj.Tree.E.father_i(del_idx_Tree_E) = [];
            obj.cost.isTreeSame(obj.Tree);
            obj = obj.updateTreeIndices();
            
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
            obj.cost.isTreeSame(obj.Tree);
        end
        
        function [Samples, volume_Xf] = Sample(obj, m, c)
            % 拒绝采样版本
            Samples = zeros(m, obj.demension);
            dem = obj.demension;
            root = obj.x_root;
            goal = obj.x_goal;
            obs = obj.obstacle;
            valid_samples_count = 0; % 用于计算体积的有效样本数
            
            % 使用 parfor 生成样本并估算体积
            for i = 1:m % parfor
                valid_sample = false;
                while ~valid_sample
                    rand_point = rand(1, dem); % 生成随机点（n维行向量）
                    if norm(rand_point - root) + norm(rand_point - goal) <= c && ~obs.isPointInside(rand_point)
                        Samples(i, :) = rand_point; % 添加行向量到矩阵
                        valid_sample = true;
                        valid_samples_count = valid_samples_count + 1; % 增加有效样本计数
                    end
                end
            end
            
            % 估算体积
            volume_Xf = valid_samples_count / m;
        end

        function obj = ExpandVertex(obj)
            % [Alg.2]
            % Pop the best vertex from Q.V

            % 并行运算临时变量
            c = obj.cost;
            Tree_copy = obj.Tree;
            tempV = [];
            tempX = [];

            [~, index] = c.BestValue(obj.Q,'V',obj.Tree);
            v = obj.Q.V(index,:);
            obj.Q.V(index,:) = [];
            
            X_near = obj.Near(v, obj.X_samples);
            start = obj.x_root;
            goal = obj.x_goal;
            [n, ~] = size(X_near);

            % QE ←+ (v, x) ∈ (V x X_near) 
            for i = 1:n% parfor
                x = X_near(i,:);
                if norm(v-start)+norm(v-x)+norm(x-goal) < c.gT(goal, Tree_copy)
                        tempV = [tempV; v];
                        tempX = [tempX; x];
                end
            end
            
            obj.Q.E.v = [obj.Q.E.v; tempV];
            obj.Q.E.x = [obj.Q.E.x; tempX];

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

        function obj = ExpandEdge(obj)
            % Pop the best edge from Q.E
            [~, index] = obj.cost.BestValue(obj.Q,'E',obj.Tree);
            v = obj.Q.E.v(index,:);
            x = obj.Q.E.x(index,:);
            obj.Q.E.v(index,:) = [];
            obj.Q.E.x(index,:) = [];

            if obj.cost.gT(v,obj.Tree)+obj.cost.c_(v,x)+obj.cost.h_(x) < obj.cost.gT(obj.x_goal,obj.Tree)
                if obj.cost.g_(v)+obj.cost.c_(v,x)+obj.cost.h_(x) < obj.cost.gT(obj.x_goal,obj.Tree)
                    if obj.cost.gT(v,obj.Tree) + obj.cost.c(v,x) < obj.cost.gT(x,obj.Tree)
                        if ismember(x, obj.Tree.V, 'rows')
                            % x ∈ V
                            del_index = ismember(obj.Tree.E.x, x, 'rows');
                            obj.Tree.E.v(del_index,:) = [];
                            obj.Tree.E.x(del_index,:) = [];
                            obj.Tree.E.father_i(del_index) = [];
                            obj.cost.isTreeSame(obj.Tree);
                            obj = obj.updateTreeIndices();
                        else
                            % x ∉ V
                            obj.X_samples = obj.X_samples(~ismember(obj.X_samples, x, 'rows'),:);
                            obj.Tree.V = [obj.Tree.V; x];
                            obj.cost.isTreeSame(obj.Tree);
                            obj.Q.V = [obj.Q.V; x];
                        end
                        obj.Tree.E.v = [obj.Tree.E.v; v];
                        obj.Tree.E.x = [obj.Tree.E.x; x];
                        [~,father_index] = ismember(v, obj.Tree.E.x, 'rows');
                        obj.Tree.E.father_i(end+1) =  father_index;
                        obj.cost.isTreeSame(obj.Tree);

                        n = size(obj.Q.E.v, 1);
                        rows_to_remove = false(n, 1); % 初始化一个逻辑数组来标记要移除的行
                        for i = 1:n% parfor
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
            while ~isempty(current)
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
            view(10, 30);
            hold off;
        end

        function r = Radius(obj, p, a)
            % Lebesgue measure
            n = obj.demension;
            ball_Lebesgue = pi^(n/2) / gamma(n/2 + 1);
            r = 2*a * (1+1/n)^(1/n) * (obj.volume_Xf/ball_Lebesgue)^(1/n) * (log(p)/p)^(1/n);
        end

        function obj = updateTreeIndices(obj)
            % 遍历每个节点，更新其父节点的索引
            for i = 1:size(obj.Tree.E.x)
                % 获取当前节点的父节点
                current = obj.Tree.E.v(i,:);
                [~,father_index] = ismember(current, obj.Tree.E.x, 'rows');
                obj.Tree.E.father_i(i) = father_index;
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Path_Simplification(obj)
            if isempty(obj.Tree.E.x)
                return
            end 

            path = obj.Path();
            fprintf('cost after simplification = %f\n', obj.calculatePathLength(path));
            % simplifiedPath = obj.douglasPeucker(path,  0.01);% douglasPeucker
            simplifiedPath = obj.FSPS(path);% Forward sequential path simplification 
            fprintf('cost after simplification = %f\n', obj.calculatePathLength(simplifiedPath));
            obj.Tree.E = struct('v',[],'x',[],'father_i',[]);
            for i = 1:size(simplifiedPath,1)-1
                obj.Tree.E.v(i,:) =  simplifiedPath(i,:);
                obj.Tree.E.x(i,:) =  simplifiedPath(i+1,:);
                obj.Tree.E.father_i(i) = i-1;
            end
        end
        function simplifiedPath = douglasPeucker(obj, path, epsilon)
            % 如果路径上的点少于3个，无需简化
            if size(path, 1) < 3
                simplifiedPath = path;
                return;
            end
            
            % 找到距离基准线段最远的点
            maxDistance = 0;
            index = 0;
            for i = 2:size(path, 1) - 1
                distance = obj.perpDist(path(i, :), path(1, :), path(end, :));
                if distance > maxDistance
                    maxDistance = distance;
                    index = i;
                end
            end

            % 如果最大距离小于阈值，检查路径是否与障碍物相交
            if maxDistance < epsilon
               if obj.cost.c(path(1, :), path(end, :)) < inf
                    simplifiedPath = [path(1, :); path(end, :)];
                else
                    simplifiedPath = path;
                end
            else
                % 否则，递归地对两部分路径进行简化
                firstPart = douglasPeucker(obj, path(1:index, :), epsilon);
                secondPart = douglasPeucker(obj, path(index:end, :), epsilon);
                simplifiedPath = [firstPart(1:end-1, :); secondPart];
            end
        end
        function distance = perpDist(obj, point, lineStart, lineEnd)
            % 计算点到线段的垂直距离（三维）
            lineVec = lineEnd - lineStart;
            pointVec = point - lineStart;
            lineLen = norm(lineVec);
            projLen = dot(pointVec, lineVec) / lineLen;
            projVec = (projLen / lineLen) * lineVec;
            perpVec = pointVec - projVec;
            distance = norm(perpVec);
        end

        function totalLength = calculatePathLength(obj, points)
            % 计算路径的总长度
            % points: 一个N x 3的矩阵，每行代表一个点的x, y, z坐标

            % 计算每两个相邻点之间的距离
            differences = diff(points, 1, 1); % 计算相邻点之间的差异
            distances = sqrt(sum(differences.^2, 2)); % 计算每段的欧几里得距离

            % 计算总长度
            totalLength = sum(distances);
        end

        function simplified_path = FSPS(obj,original_path)
            % 初始化简化后的路径
            simplified_path = original_path(1, :);

            i = 1;
            while i < size(original_path, 1)
                % 从当前节点开始，尝试直接连接到后面的节点
                for j = size(original_path, 1):-1:i+1
                    if obj.cost.c(original_path(i, :), original_path(j, :)) ~= inf
                        % 如果两点之间没有障碍物，将终点添加到简化路径中
                        simplified_path = [simplified_path; original_path(j, :)];
                        i = j; % 更新当前节点索引
                        break;
                    end
                end
                % 如果所有后续节点都无法直接连接，则移动到下一个节点
                if i < size(original_path, 1) && isequal(simplified_path(end, :), original_path(i, :))
                    i = i + 1;
                    simplified_path = [simplified_path; original_path(i, :)];
                end
            end
        end
    end % end methods
end % end classdef