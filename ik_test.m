clear all;
clc;
close all;
% 加载或创建一个机器人模型
load('robot_ABB1200.mat');
robot = SerialLink([ML2 ML3 ML4 ML5 ML6 ML7], 'name', 'ABB1200');
% 定义障碍物
center = [250 250 250]; % 中心点坐标
length = 100; % 立方体的长度
width = 100;  % 立方体的宽度
height = 100; % 立方体的高度
obstacle = Obstacle(center, length, width, height);

% 定义一组测试的关节角度
init_ang = [0, 0, 0, 0, 0, 0];
targ_ang = [pi/2, pi/4, pi/3, pi/6, pi/8, pi/2];
step = 50;

% 使用jtraj生成关节角度、速度、加速度
[q, qd, qdd] = jtraj(init_ang, targ_ang, step);

% 使用fkine生成末端轨迹
% 计算末端轨迹
T = robot.fkine(q);
nT = T.T;

% 使用关节角度、速度、加速度和末端轨迹创建Trajectory对象
trajectory1 = Trajectory(q, qd, qdd, nT);

% 从trajectory1中提取positions，并使用它和机器人模型创建新的Trajectory对象
trajectory2 = Trajectory(trajectory1.positions, robot);

figure;
hold on;
title('正运动学末端轨迹');
obstacle.plotObstacle();
trajectory1.plotTrajectory();
% 绘制第一个轨迹
%robot.plot(trajectory1.q, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);
hold off;

figure;
hold on;
title('逆运动学末端轨迹');
obstacle.plotObstacle();
trajectory2.plotTrajectory();
% 绘制第二个轨迹
%robot.plot(trajectory2.q, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);
hold off;
% rrt*函数测试
start_point = [225;-1.83697019872103e-14;177];
goal_point = [9.56708580912675;296.022756199211;539.942857714129];
trajectory3 = rrt_star_planning(start_point, goal_point, obstacle, robot);

figure;
hold on;
title('rrt*末端轨迹');
obstacle.plotObstacle();
trajectory3.plotTrajectory();
% 绘制第三个轨迹
%robot.plot(trajectory3.q, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);
hold off;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all;
% clc;
% close all;
% % 加载或创建一个机器人模型
% load('robot_ABB1200.mat');
% robot = SerialLink([ML2 ML3 ML4 ML5 ML6 ML7], 'name', 'ABB1200');
% % 定义障碍物
% center = [0.25 0.25 0.25]; % 中心点坐标
% length = 0.1; % 立方体的长度
% width = 0.1;  % 立方体的宽度
% height = 0.1; % 立方体的高度
% obstacle = Obstacle(center, length, width, height);
% 
% % 定义一组测试的关节角度
% init_ang = [0, 0, 0, 0, 0, 0];
% targ_ang = [pi/2, pi/4, pi/3, pi/6, pi/8, pi/2];
% step = 50;
% 
% % 使用jtraj生成关节角度、速度、加速度
% [q, qd, qdd] = jtraj(init_ang, targ_ang, step);
% 
% % 使用fkine生成末端轨迹
% % 计算末端轨迹
% T = robot.fkine(q);
% nT = T.T;
% 
% % 使用关节角度、速度、加速度和末端轨迹创建Trajectory对象
% trajectory1 = Trajectory(q, qd, qdd, nT);
% 
% % 从trajectory1中提取positions，并使用它和机器人模型创建新的Trajectory对象
% % trajectory2 = Trajectory(trajectory1.positions, robot);
% 
% figure;
% % subplot(1, 2, 1);
% hold on;
% title('末端轨迹');
% obstacle.plotObstacle();
% trajectory1.plotTrajectory();
% % 绘制第一个轨迹
% robot.plot(trajectory1.q, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);
% hold off;
% 
% % subplot(1, 2, 2);
% hold on;
% title('末端轨迹');
% obstacle.plotObstacle();
% % trajectory2.plotTrajectory();
% % 绘制第二个轨迹
% % robot.plot(trajectory2.q, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);
% hold off;
% 
% % 逆解测试
% % 使用ikine计算逆解
% q_inv = robot.ikine(nT, 'mask', [1 1 1 0 0 0]);
% 
% % 使用plot绘制逆解的轨迹
% figure;
% title('逆解轨迹');
% robot.plot(q_inv, 'noshadow', 'nojaxes', 'nojvec', 'nojoints', 'nobase', 'notiles', 'delay', 0.1);