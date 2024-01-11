clc;clear;close all;

% 定义环境参数
x_root = [0.3 0.9 0.45]; % 起点坐标（列向量）
x_goal = [0.6 0.5 0.45]; % 目标点坐标（列向量）

% 定义障碍物
center = [0.4 0.75 0.45]; % 中心点坐标
le = 0.1; % 立方体的长度
width = 0.2;  % 立方体的宽度
height = 0.1; % 立方体的高度
obs1 = aObstacle(center, le, width, height);

center = [0.5 0.55 0.45]; % 中心点坐标
le = 0.3; % 立方体的长度
width = 0.02;  % 立方体的宽度
height = 0.05; % 立方体的高度
obs2 = aObstacle(center, le, width, height);

obstacle = Obstacle;
obstacle = obstacle.addObstacle(obs1);
obstacle = obstacle.addObstacle(obs2);

total_time = 0;
nums = 1;
for i = 1:nums
    tic;

    bit = BIT_star_rebuild(obstacle, x_root, x_goal ,3 , 2500);
    path = bit.Solution();

    elapsed_time = toc;
    total_time = total_time + elapsed_time;
end
average_time = total_time / nums;
disp(['Average Time: ', num2str(average_time), ' seconds']);

figure;
hold on;
axis equal;
grid on;

% 绘制障碍物
obstacle.plotObstacle();

% 绘制采样点
plot3(path(:,1), path(:,2), path(:,3), 'r.-');
% 标记起点和终点
plot3(x_root(1), x_root(2), x_root(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x_goal(1), x_goal(2), x_goal(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

xlabel('X');
ylabel('Y');
zlabel('Z');
title('BIT*');
hold off;
% function X_rand = randSample(x_root, X_goal, c_sol, m, obstacle, dist)
%     X_rand = []; % 初始化为空矩阵
%     while size(X_rand, 2) < m
%         rand_point = rand(3,1); % 生成随机点（列向量）
%         if dist(rand_point, x_root) + dist(rand_point, X_goal) <= c_sol && ~obstacle.isVectorIntersectingObstacle(rand_point, X_goal)
%             X_rand = [X_rand, rand_point]; % 添加列向量到矩阵
%         end
%     end    
% end
% function X_rand = samplePointsInSphere(n, m, x_root, X_goal, c_sol, obstacle)
%     % n是空间的维度
%     % m是要生成的点的数量
%     % 生成单位向量
%     X_rand = [];
%     while size(X_rand, 2) < m
%         normalVals = randn(n, m); % 生成n*m个标准正态分布的随机数
%         unitVectors = bsxfun(@rdivide, normalVals, sqrt(sum(normalVals.^2, 1)));
% 
%         % 生成随机半径
%         radii = ((norm(x_root-X_goal)/1.2)*rand(1, m)).^(1/n);
% 
%         % 调整每个点的大小以匹配其半径
%         points = bsxfun(@times, unitVectors, radii) + (x_root + X_goal)/2;
%         for i = 1:length(points)
%             rand_point = points(:,i);
%             if size(X_rand, 2) == m
%                 break
%             end
%             if norm(rand_point - x_root) + norm(rand_point - X_goal) <= c_sol && ~obstacle.isVectorIntersectingObstacle(rand_point, X_goal)
%                 X_rand = [X_rand, rand_point]; % 添加列向量到矩阵
%             end
%         end        
%     end   
% end