% 定义环境参数
x_root = [0.3; 0.6; 0.5]; % 起点坐标（列向量）
X_goal = [0.5; 0.6; 0.3]; % 目标点坐标（列向量）
c_sol = 0.3; % 当前最优路径长度

% 距离函数定义
dist = @(x, y) norm(x - y);

% 定义障碍物
center = [0.3 0.65 0.25]; % 中心点坐标
length = 0.1; % 立方体的长度
width = 0.1;  % 立方体的宽度
height = 0.1; % 立方体的高度
obstacle = Obstacle(center, length, width, height);

% % 测试 randSample 函数
% m = 10000; % 采样点个数
% X_rand = randSample(x_root, X_goal, c_sol, m, obstacle, dist);

X_sol = BIT_star(x_root, X_goal, obstacle);
% % 绘制障碍物和采样点
% figure;
% hold on;
% axis equal;
% grid on;
% 
% % 绘制障碍物
% obstacle.plotObstacle();
% 
% % 绘制采样点
% for i = 1:numel(X_rand)
%     plot3(X_rand{i}(1), X_rand{i}(2), X_rand{i}(3), 'r.');
% end
% 
% % 标记起点和终点
% plot3(x_root(1), x_root(2), x_root(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% plot3(X_goal(1), X_goal(2), X_goal(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('informed set');
% hold off;

