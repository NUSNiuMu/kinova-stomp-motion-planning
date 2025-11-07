function [Stheta, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world)
% 计算离散轨迹每一点的局部代价 Stheta 与整条轨迹代价 Qtheta
% 代价 = 避障(q_obs) + 末端到目标的指数代价(q_goalexp)
% 不在此处累加全局平滑项 (1/2 * theta^T R theta)

[~, nDiscretize] = size(theta);
qo_cost  = zeros(1, nDiscretize);   % 障碍物代价
qgoalexp = zeros(1, nDiscretize);   % 末端到目标位置的指数代价

% ---- 从 base 取目标与末端名称（与主脚本一致）----
endEffector = evalin('base','endEffector');   % 例如 "EndEffector_Link"
T_goal      = evalin('base','taskFinal');     % 4x4 齐次变换
p_goal      = T_goal(1:3,4);                  % 目标位置

% 指数代价参数（可调）
w_obs  = 1.0e4;     % 避障权重（与原来量级一致）
w_goal = 30;       % 末端指数项权重（建议 100~500 试）
gamma  = 5.0;       % 指数“尖锐度”，越大越强烈（建议 3~10）

% ---------- i = 1 ----------
[X, ~] = updateJointsWorldPosition(robot_struct, theta(:,1));
[sphere_centers, radi] = stompRobotSphere(X);
vel_cart = zeros(length(sphere_centers), 1);
qo_cost(1) = stompObstacleCost(sphere_centers, radi, voxel_world, vel_cart);

% 计算末端位置并给指数代价
qConf = robot_struct.homeConfiguration;
qcell = num2cell(theta(:,1));
[qConf.JointPosition] = qcell{:};
T_ee = getTransform(robot_struct, qConf, endEffector);
p_ee = T_ee(1:3,4);
d2   = sum((p_ee - p_goal).^2);               % 平滑：用平方距离
qgoalexp(1) = w_goal * exp(gamma * d2);

% ---------- i = 2..n ----------
for i = 2:nDiscretize
    prev_centers = sphere_centers;

    [X, ~] = updateJointsWorldPosition(robot_struct, theta(:,i));
    [sphere_centers, radi] = stompRobotSphere(X);

    % 避障：保持原有“近似速度权重”
    vel_cart = vecnorm(prev_centers - sphere_centers, 2, 2);
    qo_cost(i) = stompObstacleCost(sphere_centers, radi, voxel_world, vel_cart);

    % 末端指数代价
    qcell = num2cell(theta(:,i));
    [qConf.JointPosition] = qcell{:};
    T_ee = getTransform(robot_struct, qConf, endEffector);
    p_ee = T_ee(1:3,4);
    d2   = sum((p_ee - p_goal).^2);
    qgoalexp(i) = w_goal * exp(gamma * d2);
end

% ---------- 局部代价与本函数的全局和 ----------
Stheta = w_obs * qo_cost + qgoalexp;
Qtheta = sum(Stheta);     % 注意：不含 (1/2)*theta^T R theta

end

% ====== 辅助：兼容 struct / 对象（如你需要保留可以放这儿）======
function tf = hasFieldOrProp(obj, name)
    if isstruct(obj), tf = isfield(obj, name);
    else,              tf = isprop(obj, name);
    end
end

function val = getFieldOrProp(obj, name)
    if isstruct(obj), val = obj.(name);
    else,              val = obj.(name);
    end
end

function h = plotSpheres(centers, radi, ax)
if nargin < 3 || isempty(ax)
    f = figure('Color','w'); ax = axes('Parent',f); view(ax,3);
    grid(ax,'on'); axis(ax,'equal'); hold(ax,'on');
end
theta = linspace(0,2*pi,24);
phi   = linspace(0,pi,12);
[uu,vv] = meshgrid(theta,phi);
ux = sin(vv).*cos(uu); uy = sin(vv).*sin(uu); uz = cos(vv);
h = gobjects(size(radi));
for i = 1:numel(radi)
    Xs = centers(i,1) + radi(i)*ux;
    Ys = centers(i,2) + radi(i)*uy;
    Zs = centers(i,3) + radi(i)*uz;
    h(i) = surf(ax, Xs, Ys, Zs, 'FaceAlpha',0.25, 'EdgeColor','none');
end
end
