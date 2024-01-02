clc;
clear;
% 定义环境参数
x_root = [0.3;0.6;0.5]; % 起点坐标（列向量）
X_goal = [0.5;0.9;0.3]; % 目标点坐标（列向量）
c_sol = 0.5; % 当前最优路径长度
Q.v = [x_root];
Q.x = [X_goal];
% 距离函数定义
dist = @(x, y) norm(x - y);

% 定义障碍物
center = [0.3 0.9 0.55]; % 中心点坐标
le = 0.1; % 立方体的长度
width = 0.1;  % 立方体的宽度
height = 0.1; % 立方体的高度
obstacle = Obstacle(center, le, width, height);

% % 测试 randSample 函数
m = 2500; % 采样点个数
points = samplePointsInSphere(3, m, x_root, X_goal, c_sol, obstacle);
X_rand = points;
X_rand2 = randSample(x_root, X_goal, c_sol, m, obstacle, dist);
%bit = BIT_star(x_root, X_goal, obstacle);
%X_sol = bit.BIT();
% 绘制障碍物和采样点
figure;
hold on;
axis equal;
grid on;

% 绘制障碍物
obstacle.plotObstacle();

% 绘制采样点
for i = 1:length(X_rand)
    plot3(X_rand(1,i), X_rand(2,i), X_rand(3,i), 'r.');
end

% 标记起点和终点
plot3(x_root(1), x_root(2), x_root(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(X_goal(1), X_goal(2), X_goal(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');

xlabel('X');
ylabel('Y');
zlabel('Z');
title('informed set');
hold off;
function X_rand = randSample(x_root, X_goal, c_sol, m, obstacle, dist)
    X_rand = []; % 初始化为空矩阵
    while size(X_rand, 2) < m
        rand_point = rand(3,1); % 生成随机点（列向量）
        if dist(rand_point, x_root) + dist(rand_point, X_goal) <= c_sol && ~obstacle.isVectorIntersectingObstacle(rand_point, X_goal)
            X_rand = [X_rand, rand_point]; % 添加列向量到矩阵
        end
    end    
end
function X_rand = samplePointsInSphere(n, m, x_root, X_goal, c_sol, obstacle)
    % n是空间的维度
    % m是要生成的点的数量
    % 生成单位向量
    X_rand = [];
    while size(X_rand, 2) < m
        normalVals = randn(n, m); % 生成n*m个标准正态分布的随机数
        unitVectors = bsxfun(@rdivide, normalVals, sqrt(sum(normalVals.^2, 1)));
    
        % 生成随机半径
        radii = ((norm(x_root-X_goal)/1.2)*rand(1, m)).^(1/n);
    
        % 调整每个点的大小以匹配其半径
        points = bsxfun(@times, unitVectors, radii) + (x_root + X_goal)/2;
        for i = 1:length(points)
            rand_point = points(:,i);
            if size(X_rand, 2) == m
                break
            end
            if norm(rand_point - x_root) + norm(rand_point - X_goal) <= c_sol && ~obstacle.isVectorIntersectingObstacle(rand_point, X_goal)
                X_rand = [X_rand, rand_point]; % 添加列向量到矩阵
            end
        end        
    end   
end