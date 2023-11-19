clear all
clc

load('robot_ABB1200.mat');
% 创建机器人模型
robot = SerialLink([ML2 ML3 ML4 ML5 ML6 ML7],'name','ABB1200');

% 定义立方体障碍物的顶点
obstacle_vertices = [
    200 200 200; % 顶点1
    200 300 200; % 顶点2
    300 300 200; % 顶点3
    300 200 200; % 顶点4
    200 200 300; % 顶点5
    200 300 300; % 顶点6
    300 300 300; % 顶点7
    300 200 300; % 顶点8
];

% 定义立方体的边
obstacle_edges = [
    1 2;
    2 3;
    3 4;
    4 1;
    5 6;
    6 7;
    7 8;
    8 5;
    1 5;
    2 6;
    3 7;
    4 8;
];

% 定义初始和目标关节角度
init_ang = [0, 0, 0, 0, 0, 0];
targ_ang = [pi/2, pi/4, pi/3, pi/6, pi/8, pi/2];

% 使用 jtraj 生成轨迹
step = 50; % 样本数
[q, qd, qdd] = jtraj(init_ang, targ_ang, step);

% 末端轨迹
T = robot.fkine(q);
nT = T.T;

% 绘制末端轨迹和机器人动画
f = figure;
subplot(2, 2, [2,4]);
hold on;
title('末端轨迹');
% 绘制立方体障碍物
for edge = obstacle_edges'
    plot3([obstacle_vertices(edge(1), 1), obstacle_vertices(edge(2), 1)], ...
          [obstacle_vertices(edge(1), 2), obstacle_vertices(edge(2), 2)], ...
          [obstacle_vertices(edge(1), 3), obstacle_vertices(edge(2), 3)], 'k-', 'LineWidth', 2);
end
plot3(squeeze(nT(1, 4, :)), squeeze(nT(2, 4, :)), squeeze(nT(3, 4, :)));

robot.plot(q, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0);
% 绘制关节角度、速度和加速度变化
subplot(2, 2, 1);
plot(q);
hold on;
plot(qd);
plot(qdd);
legend('关节角度', '关节速度', '关节加速度');


subplot(2, 2, 3);
% 重复绘制立方体障碍物
hold on;
view(3);
for edge = obstacle_edges'
    plot3([obstacle_vertices(edge(1), 1), obstacle_vertices(edge(2), 1)], ...
          [obstacle_vertices(edge(1), 2), obstacle_vertices(edge(2), 2)], ...
          [obstacle_vertices(edge(1), 3), obstacle_vertices(edge(2), 3)], 'k-', 'LineWidth', 2);
end
% 重复绘制末端轨迹
plot3(squeeze(nT(1, 4, :)), squeeze(nT(2, 4, :)), squeeze(nT(3, 4, :)), 'r-');
hold off;
% 定义权重
weights = [1, 1, 1];

% 调用函数
[loss_distance, loss_smoothness, loss_time, total_loss] = calculate_losses(robot, q, qd, qdd, obstacle_vertices, 0.1, weights);

% 输出损失值
fprintf('距离损失: %f\n', loss_distance);
fprintf('平滑度损失: %f\n', loss_smoothness);
fprintf('时间损失: %f\n', loss_time);
fprintf('总损失: %f\n', total_loss);


function [loss_distance, loss_smoothness, loss_time, total_loss] = calculate_losses(robot, q, qd, qdd, obstacle_vertices, time_per_step, weights)
    % 计算与障碍物的最短距离损失
    step = size(q, 1);
    T = robot.fkine(q);
    nT = T.T;
    distances = zeros(step, 1);
    for i = 1:step
        end_effector_pos = nT(1:3, 4, i)';
        dist_to_vertices = sqrt(sum((obstacle_vertices - end_effector_pos).^2, 2));
        distances(i) = min(dist_to_vertices);
    end
    loss_distance = sum(1 ./ distances);

    % 计算加速度平滑程度损失
    acceleration_changes = diff(qdd); % 这里的 qdd 应该是一个矩阵，行表示时间步，列表示关节
    acceleration_changes_vector = acceleration_changes(:); % 将矩阵转换成一个长向量
    std_acc_changes = std(acceleration_changes_vector); % 计算总体标准差
    loss_smoothness = std_acc_changes;

    % 计算轨迹整体运行时间损失
    total_time = step * time_per_step;
    loss_time = total_time;

    % 计算综合损失
    total_loss = weights(1) * loss_distance + weights(2) * loss_smoothness + weights(3) * loss_time;
end
