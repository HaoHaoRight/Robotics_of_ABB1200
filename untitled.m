clc;clear;
% 定义障碍物的中心点、长度、宽度和高度
center = [0, 0, 0]; % 障碍物中心在原点
length = 10;        % 长度为10
width = 10;         % 宽度为10
height = 10;        % 高度为10

% 创建障碍物对象
obstacle = Obstacle(center, length, width, height);

% 定义几个测试点
points = [
    0, 0, 0;    % 中心点，应在障碍物内
    5, 5, 5;    % 边界上的点，应在障碍物内
    6, 6, 6;    % 外部点，应在障碍物外
    -5, -5, -5; % 边界上的另一点，应在障碍物内
    -6, -6, -6  % 另一个外部点，应在障碍物外
];

% 检查每个点是否在障碍物内
% 检查每个点是否在障碍物内
for i = 1:size(points, 1)
    point = points(i, :);
    inside = obstacle.isPointInside(point);
    if inside
        status = ''; % 在障碍物内
    else
        status = '不'; % 不在障碍物内
    end
    fprintf('点 (%.2f, %.2f, %.2f) %s在障碍物内部。\n', point(1), point(2), point(3), status);
end
