% FILEPATH: /c:/Users/a1606/Documents/robot/Robotics_of_ABB1200/Trajectory.m
classdef Trajectory
    properties
        q;      % 关节角度
        qd;     % 关节速度
        qdd;    % 关节加速度
        nT;     % 末端轨迹
    end
    
    methods
        function obj = Trajectory(q, qd, qdd, nT)
            % 构造函数，初始化对象的属性
            % 参数:
            %   q: 关节角度
            %   qd: 关节速度
            %   qdd: 关节加速度
            %   nT: 末端轨迹
            obj.q = q;
            obj.qd = qd;
            obj.qdd = qdd;
            obj.nT = nT;
        end
    
        function plotTrajectory(obj)
            % 绘制轨迹
            plot3(squeeze(obj.nT(1, 4, :)), squeeze(obj.nT(2, 4, :)), squeeze(obj.nT(3, 4, :)), 'r-');
        end

        function plotJointStates(obj)
            % 绘制关节角度、速度和加速度变化
            subplot(2, 2, 1);
            plot(obj.q);
            hold on;
            plot(obj.qd);
            plot(obj.qdd);
            hold off;
            legend('关节角度', '关节速度', '关节加速度');
        end
        
        function [loss_distance, loss_smoothness, loss_time, total_loss] = calculateLosses(obj, obstacle, time_per_step, weights, epsilon)
            % 计算损失
            % 参数:
            %   obstacle: 障碍物对象
            %   time_per_step: 每个步骤的时间
            %   weights: 损失权重
            %   epsilon: 避免除以零的小量
            % 返回值:
            %   loss_distance: 距离损失
            %   loss_smoothness: 平滑度损失
            %   loss_time: 时间损失
            %   total_loss: 综合损失
            step = size(obj.q, 1);
            distances = zeros(step, 1);
            for i = 1:step
                end_effector_pos = obj.nT(1:3, 4, i)';
                % 使用障碍物类中的距离计算方法
                distances(i) = obstacle.distanceToPoint(end_effector_pos);
            end
        
            % 使用势能场方法
            loss_distance = sum(1 ./ (distances.^2 + epsilon));
        
            % 对于过于接近障碍物的情况，增加额外的惩罚
            close_penalty = sum(distances < 10) * 1;
            loss_distance = loss_distance + close_penalty;
        
            % 计算加速度平滑程度损失
            acceleration_changes = diff(obj.qdd);
            acceleration_changes_vector = acceleration_changes(:);
            std_acc_changes = std(acceleration_changes_vector);
            loss_smoothness = std_acc_changes;

            % 计算轨迹整体运行时间损失
            total_time = step * time_per_step;
            loss_time = total_time;
        
            % 计算综合损失
            total_loss = weights(1) * loss_distance + weights(2) * loss_smoothness + weights(3) * loss_time;
        end
    end
end