
classdef Trajectory
    properties
        q;      % 关节角度
        qd;     % 关节速度
        qdd;    % 关节加速度
        nT;     % 末端轨迹
        % 提取所有位移部分
        % 将结果重新整理为一个矩阵，其中每一列代表一个时间点的位移
        positions;
        
        loss_distance;      % 距离损失.
        loss_smoothness;    % 平滑度损失
        loss_time;          % 时间损失
        total_loss;         % 综合损失
        time_per_step = 0.1;% 每个步骤的时间
        weights = [1, 1, 1];% 损失权重
        epsilon = 1e-2;     % 势能
    end
    
    methods

        function obj = Trajectory(x1, x2, x3, x4)
            % Trajectory 构造函数，初始化对象的属性
            % 参数类型1:
            %   q: 关节角度
            %   qd: 关节速度
            %   qdd: 关节加速度
            %   nT: 末端轨迹
            %   positions: 末端轨迹(点形式)
            % 参数类型2:
            %   positions: 末端轨迹(点形式)
            %   robot: 机器人模型
                if nargin == 4
                    obj.q = x1;
                    obj.qd = x2;
                    obj.qdd = x3;
                    obj.nT = x4;
                    obj.positions = reshape(obj.nT(1:3, 4, :), 3, []);
                elseif nargin == 2
                    obj.positions = x1;
                    obj = computeInverseKinematics(obj, x2);
                else
                    error('Invalid number of input arguments');
                end
        end
        

        function plotTrajectory(obj)
            % plotTrajectory 绘制轨迹
            plot3(obj.positions(1, :), obj.positions(2, :), obj.positions(3, :), 'r-');
        end

        function plotJointStates(obj)
            % plotJointStates 绘制关节角度、速度和加速度变化
            subplot(2, 2, 1);
            plot(obj.q);
            hold on;
            plot(obj.qd);
            plot(obj.qdd);
            hold off;
            legend('关节角度', '关节速度', '关节加速度');
        end
        
        function obj = calculateLosses(obj, obstacle)
            % calculateLosses 计算损失
            % 参数:
            %   obstacle: 障碍物对象
            % 返回值:
            %   loss_dist: 距离损失
            %   loss_smooth: 平滑度损失
            %   loss_t: 时间损失
            %   total: 综合损失
            step = size(obj.q, 1);
            distances = zeros(step, 1);
            for i = 1:step
                end_effector_pos = obj.nT(1:3, 4, i)';
                % 使用障碍物类中的距离计算方法
                distances(i) = obstacle.distanceToPoint(end_effector_pos);
            end

            % 使用势能场方法
            loss_dist = sum(1 ./ (distances.^2 + obj.epsilon));

            % 对于过于接近障碍物的情况，增加额外的惩罚
            close_penalty = sum(distances < 10) * 1;
            loss_dist = loss_dist + close_penalty;

            % 计算加速度平滑程度损失
            acceleration_changes = diff(obj.qdd);
            acceleration_changes_vector = acceleration_changes(:);
            std_acc_changes = std(acceleration_changes_vector);
            loss_smooth = std_acc_changes;

            % 计算轨迹整体运行时间损失
            total_time = step * obj.time_per_step;
            loss_t = total_time;

            % 计算综合损失
            total = obj.weights(1) * loss_dist + obj.weights(2) * loss_smooth + obj.weights(3) * loss_t;
            obj.loss_distance = loss_dist;
            obj.loss_smoothness = loss_smooth;
            obj.loss_time = loss_t;
            obj.total_loss = total;
        end
        function obj = computeInverseKinematics(obj, robot)
            % 计算逆运动学
            % 参数:
            %   robot - 机器人模型
    
            n = size(obj.positions, 2);  % 轨迹中的点的数量
            obj.q = zeros(n, robot.n);   % 初始化关节角度矩阵
    
            for i = 1:n
                T(1,i) = SE3(obj.positions(:, i));  % 创建SE3对象
            end
            obj.q = robot.ikine(T, 'mask', [1 1 1 0 0 0]);  % 计算每个点的逆运动学

            for i = 1:n
                T = robot.fkine(obj.q(i, :));  % 计算每个时间点的正向运动学
                obj.nT(:, :, i) = T;  % 存储位姿
            end
            % 接下来，可以计算关节速度和加速度
            obj.qd = diff(obj.q) / obj.time_per_step;
            obj.qdd = diff(obj.qd) / obj.time_per_step;
        end

    end
end