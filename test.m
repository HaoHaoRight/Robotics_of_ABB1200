clear all;
clc;
close all;
% 加载机械臂模型数据
load('robot_ABB1200.mat');
robot = SerialLink([ML2 ML3 ML4 ML5 ML6 ML7], 'name', 'ABB1200');

% 定义障碍物
center = [0.25 0.25 0.25]; % 中心点坐标
length = 0.1; % 立方体的长度
width = 0.1;  % 立方体的宽度
height = 0.1; % 立方体的高度
obstacle = Obstacle(center, length, width, height);

% 定义初始和目标关节角度，生成轨迹
init_ang = [0, 0, 0, 0, 0, 0];
targ_ang = [pi/2, pi/4, pi/3, pi/6, pi/8, pi/2];
step = 50;
[q, qd, qdd] = jtraj(init_ang, targ_ang, step);

% 计算末端轨迹
T = robot.fkine(q);
nT = T.T;
% 末端轨迹矩阵 nT 的结构        
% 末端轨迹矩阵的每一行都是一个 4x4 的矩阵，表示末端执行器的位姿。
% 旋转部分：一个 3x3 的矩阵，表示末端执行器的方向。
% 位移部分：一个 3x1 的向量，表示末端执行器的位置。
% 最后一行：通常是 [0 0 0 1]，用于保持矩阵的数学性质。

% 创建轨迹对象
trajectory = Trajectory(q, qd, qdd, nT);

% 绘制末端轨迹和机器人动画
f = figure;
subplot(2, 2, [2,4]);
hold on;
title('末端轨迹');
obstacle.plotObstacle();
trajectory.plotTrajectory();
robot.plot(trajectory.q, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);
hold off;

% 绘制关节角度、速度和加速度变化
subplot(2, 2, 1);
trajectory.plotJointStates();

% 绘制立方体障碍物和末端轨迹
subplot(2, 2, 3);
hold on;
view(3);
% 绘制立方体障碍物
obstacle.plotObstacle();
% 绘制末端轨迹
trajectory.plotTrajectory();
hold off;
% 输入参数：障碍物、时间步长、权重、势能
trajectory = trajectory.calculateLosses(obstacle);

% 输出损失值
fprintf('距离损失: %f\n', trajectory.loss_distance);
fprintf('平滑度损失: %f\n', trajectory.loss_smoothness);
fprintf('时间损失: %f\n', trajectory.loss_time);
fprintf('总损失: %f\n', trajectory.total_loss);