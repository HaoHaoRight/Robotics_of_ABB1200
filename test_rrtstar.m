clear all;
clc;
close all;

% 加载机械臂模型数据
load('robot_ABB1200.mat');
robot = SerialLink([ML2 ML3 ML4 ML5 ML6 ML7], 'name', 'ABB1200');

% 定义障碍物
center = [250 250 250]; % 中心点坐标
length = 100; % 立方体的长度
width = 100;  % 立方体的宽度
height = 100; % 立方体的高度
obstacle = Obstacle(center, length, width, height);

% 定义初始和目标关节角度，生成轨迹
init_ang = [0, 0, 0, 0, 0, 0];
targ_ang = [pi/2, pi/4, pi/3, pi/6, pi/8, pi/2];
step = 50;
[q, qd, qdd] = jtraj(init_ang, targ_ang, step);

% 计算末端轨迹
T = robot.fkine(q);
nT = T.T;

% 创建轨迹对象
trajectory = Trajectory(q, qd, qdd, nT);

% 定义权重并计算初始损失
weights = [1, 1, 0.1]; % 可以根据需要调整权重
epsilon = 1e-4; % 用于计算梯度的微小扰动
time_per_step = 0.1; % 时间步长
[loss_distance_before, loss_smoothness_before, loss_time_before, total_loss_before] = trajectory.calculateLosses(obstacle, time_per_step, weights, epsilon);

% 梯度下降参数
learning_rate = 0.09; % 学习率
max_iter = 30; % 最大迭代次数

% 执行梯度下降优化并监控进度
optimized_trajectory = gradient_descent_optimization(trajectory, robot, obstacle, time_per_step, weights, epsilon, learning_rate, max_iter);

% 计算优化后的损失
[loss_distance_after, loss_smoothness_after, loss_time_after, total_loss_after] = optimized_trajectory.calculateLosses(obstacle, time_per_step, weights, epsilon);

% 输出优化前后的损失值
fprintf('优化前距离损失: %f\n', loss_distance_before);
fprintf('优化前平滑度损失: %f\n', loss_smoothness_before);
fprintf('优化前时间损失: %f\n', loss_time_before);
fprintf('优化前总损失: %f\n', total_loss_before);
fprintf('优化后距离损失: %f\n', loss_distance_after);
fprintf('优化后平滑度损失: %f\n', loss_smoothness_after);
fprintf('优化后时间损失: %f\n', loss_time_after);
fprintf('优化后总损失: %f\n', total_loss_after);

% 可视化结果
figure;
subplot(1, 2, 1);
hold on;
title('优化前轨迹');
obstacle.plotObstacle();
trajectory.plotTrajectory();
robot.plot(trajectory.q, 'trail', {'r-'}, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);
hold off;

subplot(1, 2, 2);
hold on;
title('优化后轨迹');
obstacle.plotObstacle();
optimized_trajectory.plotTrajectory();
robot.plot(optimized_trajectory.q, 'trail', {'g-'}, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);
hold off;

function path = rrt_star_planning(start_pose, goal_pose, obstacle, bounds, max_iter, delta_dist)
    % 初始化RRT树
    rrt_tree = [start_pose];
    path = [];
    delta_q = delta_dist; % 假设delta_q为步长

    for k = 1:max_iter
        % 随机采样新点
        q_rand = rand(1, length(start_pose)) .* (bounds(2:2:end) - bounds(1:2:end)) + bounds(1:2:end);

        % 寻找RRT树中离q_rand最近的节点
        [~, idx] = min(vecnorm((rrt_tree - q_rand), 2, 2));
        q_near = rrt_tree(idx, :);

        % 向q_rand方向移动一个步长以生成新节点q_new
        dir = (q_rand - q_near) / norm(q_rand - q_near);
        q_new = q_near + delta_q * dir;

        % 检查q_new是否与障碍物发生碰撞
        if obstacle.distanceToPoint(q_new) > delta_q
            % 将新节点添加到RRT树中
            rrt_tree = [rrt_tree; q_new];
        end

        % 检查是否达到目标附近
        if norm(q_new - goal_pose) < delta_q
            path = [rrt_tree; goal_pose];
            break;
        end
    end

    if isempty(path)
        warning('RRT*未能在迭代次数内找到有效路径。');
    end

    return;
end

function optimized_trajectory = gradient_descent_optimization(trajectory, robot, obstacle, time_per_step, weights, epsilon, learning_rate, max_iter)
    % 初始化优化后的轨迹
    optimized_trajectory = Trajectory(trajectory.q, trajectory.qd, trajectory.qdd, trajectory.nT);
    
    for k = 1:max_iter
        % 计算当前轨迹的损失及其关于关节角度的梯度
        [~, ~, ~, total_loss, gradient] = calculateGradients(optimized_trajectory, robot, obstacle, time_per_step, weights, epsilon);
        
        % 打印当前迭代数、总损失和梯度范数
        fprintf('轮 %d, 总损失: %f, 梯度范数: %f\n', k, total_loss, norm(gradient));
        
        % 判断梯度范数是否小于阈值
        if norm(gradient) < 1e-3
            fprintf('Optimization Converged\n');
            break;
        end
        
        % 更新关节角度
        optimized_trajectory.q = optimized_trajectory.q - learning_rate * gradient;
        
        % 重新计算末端执行器的轨迹
        T = robot.fkine(optimized_trajectory.q);
        optimized_trajectory.nT = T.T;

        % 检查是否到达目标位置
        % for step_index = 1:size(optimized_trajectory.q, 1)
        %     current_pose = T(step_index).T;
        %     if norm(current_pose(1:3, 4) - target_pose(1:3, 4)) < 10
        %         % 截断q矩阵到当前步
        %         optimized_trajectory.q = optimized_trajectory.q(1:step_index, :);
        %         optimized_trajectory.qd = optimized_trajectory.qd(1:step_index, :);
        %         optimized_trajectory.qdd = optimized_trajectory.qdd(1:step_index, :);
        %         optimized_trajectory.nT = optimized_trajectory.nT(:, :, 1:step_index);
        %         fprintf('已在 %d 步内到达目标位置。\n', step_index);
        %         return; % 退出优化函数
        %     end
        % end

        % 如果达到最大迭代次数，提前退出
        if k == max_iter
            fprintf('Reached the maximum number of iterations\n');
            break;
        end
    end
end


function [loss_distance, loss_smoothness, loss_time, total_loss, gradient] = calculateGradients(trajectory, robot, obstacle, time_per_step, weights, epsilon)
    % Compute loss and gradient
    gradient = zeros(size(trajectory.q));
    [loss_distance, loss_smoothness, loss_time, total_loss] = trajectory.calculateLosses(obstacle, time_per_step, weights, epsilon);
    
    % For each joint angle, compute the loss function's gradient with respect to that angle
    for i = 1:size(trajectory.q, 1)
        for j = 1:size(trajectory.q, 2)
            % Create a small perturbation
            dq = zeros(1, size(trajectory.q, 2));
            dq(j) = epsilon;
            
            % Perturb trajectory at step i, joint j
            perturbed_q = trajectory.q;
            perturbed_q(i, :) = perturbed_q(i, :) + dq;
            
            % Recompute the end-effector trajectory with the perturbed joint angles
            T_perturbed = robot.fkine(perturbed_q);
            nT_perturbed = T_perturbed.T;
            
            % Create a new Trajectory object with the perturbed joint angles and trajectory
            trajectory_perturbed = Trajectory(perturbed_q, trajectory.qd, trajectory.qdd, nT_perturbed);
            
            % Calculate new losses with the perturbed trajectory
            [loss_distance_perturbed, loss_smoothness_perturbed, loss_time_perturbed, total_loss_perturbed] = trajectory_perturbed.calculateLosses(obstacle, time_per_step, weights, epsilon);
            
            % Estimate the gradient as the change in total loss divided by the perturbation
            gradient(i, j) = (total_loss_perturbed - total_loss) / epsilon;
        end
    end
end
function [reached, step_count] = check_reach_goal_in_steps(robot, trajectory, goal_pose, max_steps, position_threshold)
    reached = false;
    step_count = max_steps;
    
    for i = 1:max_steps
        % 获取当前步的机器人末端执行器位置
        current_pose = robot.fkine(trajectory.q(i, :)).T;
        
        % 检查是否达到目标位置附近
        if norm(current_pose(1:3, 4) - goal_pose(1:3, 4)) < position_threshold
            reached = true;
            step_count = i;
            break;
        end
    end
end

