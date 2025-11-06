function [Stheta, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world)
% 计算离散轨迹每一点的局部代价 Stheta 与整条轨迹代价 Qtheta
% - 末端约束等权（无“终点更强”时间加权）
% - 碰撞/障碍物代价通过 stompObstacleCost 实现

[~, nDiscretize] = size(theta);
qo_cost = zeros(1, nDiscretize);   % 障碍物代价
qc_cost = zeros(1, nDiscretize);   % 末端软约束（等权重）

% ---------- 读取目标与权重（兼容 rigidBodyTree / struct） ----------
x_goal = [];
if hasFieldOrProp(robot_struct,'ee_goal') && ~isempty(getFieldOrProp(robot_struct,'ee_goal'))
    x_goal = getFieldOrProp(robot_struct,'ee_goal');  % [x;y;z] 或 1x3
elseif hasFieldOrProp(robot_struct,'goal') && ~isempty(getFieldOrProp(robot_struct,'goal'))
    x_goal = getFieldOrProp(robot_struct,'goal');
end
if ~isempty(x_goal), x_goal = x_goal(:); end

if hasFieldOrProp(robot_struct,'w_goal') && ~isempty(getFieldOrProp(robot_struct,'w_goal'))
    w_goal = getFieldOrProp(robot_struct,'w_goal');
else
    w_goal = 1e3;  % 与 1000*qo_cost 同量级，按需微调
end

% ---------- i = 1 ----------
[X, ~] = updateJointsWorldPosition(robot_struct, theta(:,1));
[sphere_centers, radi] = stompRobotSphere(X);
vel = zeros(length(sphere_centers), 1);
qo_cost(1) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);

if ~isempty(x_goal)
    x_ee = X(1:3,end);
    qc_cost(1) = w_goal * sum((x_ee - x_goal(1:3)).^2);  % 等权重，不随时间变化
else
    qc_cost(1) = 0;
end

% ---------- i = 2..n ----------
for i = 2:nDiscretize
    prev_centers = sphere_centers;

    [X, ~] = updateJointsWorldPosition(robot_struct, theta(:,i));
    [sphere_centers, radi] = stompRobotSphere(X);

    % 碰撞/障碍：保持原有速度项（近似帧间位移范数）
    vel = vecnorm(prev_centers - sphere_centers, 2, 2);
    qo_cost(i) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);

    % 末端软约束：整段轨迹等权重
    if ~isempty(x_goal)
        x_ee = X(1:3,end);
        qc_cost(i) = w_goal * sum((x_ee - x_goal(1:3)).^2);
    else
        qc_cost(i) = 0;
    end
end

% ---------- 局部代价与全局代价 ----------
Stheta = 1000*qo_cost + qc_cost;

theta_mid = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 0.5 * sum(theta_mid * R * theta_mid', "all");
end

% ====== 辅助函数：同时兼容 struct / 对象 ======
function tf = hasFieldOrProp(obj, name)
    if isstruct(obj)
        tf = isfield(obj, name);
    else
        % 对象（如 rigidBodyTree）
        tf = isprop(obj, name);
    end
end

function val = getFieldOrProp(obj, name)
    if isstruct(obj)
        val = obj.(name);
    else
        % 对象（如 rigidBodyTree）；若无该属性，上层先 hasFieldOrProp 判断
        val = obj.(name);
    end
end
