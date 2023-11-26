% 输入参数:
%   - start_point: 起始点的坐标向量
%   - goal_point: 目标点的坐标向量
%   - obstacle: 障碍物对象
%   - robot: 机器人对象
% 输出参数:
%   - trajectory: 路径规划结果的轨迹对象
function trajectory = rrt_star_planning(start_point, goal_point, obstacle, robot)
    bounds = [-600, 600; -600, 600; -600, 600]; % 定义边界
    max_iter = 600; % 定义最大迭代次数
    delta_q = 5; % 假设delta_q为步长
    % 初始化RRT树
    trajectory = Trajectory();
    trajectory.positions = start_point;
    trajectory.costs = 0; % 新增：节点的成本
    for k = 1:max_iter
        % 随机采样新点
        q_rand = (rand(1, length(start_point)) .* (bounds(2:2:end) - bounds(1:2:end)) + bounds(1:2:end)).';
        % 寻找RRT树中离q_rand最近的节点
        distances = vecnorm((trajectory.positions - q_rand), 2, 1);
        [min_distance, idx] = min(distances);
        q_near = trajectory.positions(:, idx);
        % 向q_rand方向移动一个步长以生成新节点q_new
        dir = (q_rand - q_near) / norm(q_rand - q_near);
        q_new = q_near + delta_q * dir;
        % 检查q_new是否与障碍物发生碰撞
        if obstacle.distanceToPoint(q_new) > 0.1
            % 新增：寻找在半径r内的所有节点
            r = 10;
            idx_near = find(vecnorm((trajectory.positions - q_new), 2, 1) < r);
            % 新增：找到成本最低的节点并连接
            costs_near = trajectory.costs(idx_near);
            [min_cost, idx_min] = min(costs_near + distances(idx_near));
            q_min = trajectory.positions(:, idx_near(idx_min));
            trajectory.positions = [trajectory.positions, q_new];
            trajectory.costs = [trajectory.costs, min_cost + norm(q_new - q_min)];
            % 新增：尝试重新连接树
            for i = 1:length(idx_near)
                if trajectory.costs(idx_near(i)) > trajectory.costs(end) + norm(trajectory.positions(:, idx_near(i)) - q_new)
                    trajectory.positions(:, idx_near(i)) = q_new;
                    trajectory.costs(idx_near(i)) = trajectory.costs(end) + norm(trajectory.positions(:, idx_near(i)) - q_new);
                end
            end
        end
        % 检查是否达到目标附近
        if norm(q_new - goal_point) < delta_q
            trajectory.positions = [trajectory.positions, goal_point];
            trajectory.costs = [trajectory.costs, trajectory.costs(end) + norm(goal_point - q_new)];
            break;
        end
    end
    if norm(q_new - goal_point) > delta_q
        warning('RRT*未能在迭代次数内找到有效路径。');
    end
    % trajectory = trajectory.computeInverseKinematics(robot);
    if isempty(trajectory.positions)
        warning('RRT*未能在迭代次数内找到有效路径。');
    end
end

