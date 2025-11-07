%% Construct the sphere around the robot maniputor
% It requires knowledge of the robot geometry. Here we roughly set the
% sphere radius to be 0.05m to cover the KINOVA Gen3 robot. There are 7
% joints and 8 links of KINOVA Gen3.
% INPUT:
%       X: joints (x,y,z,1) position in the world frame (Size: nJoints by 4)
% OUTPUT:
%       sphere centers and radi
function [centers, radi] = stompRobotSphere(X)
% 基于关节位姿为每段连杆布置一串固定数量的碰撞球
% 目标：保证相邻时刻生成的球数量恒定，避免后续速度计算的尺寸不匹配

% 固定参数
rad = 0.1;  % 碰撞球半径（可调），同时作为间距尺度

nJoints = size(X,1);

% 使用 persistent 缓存每段连杆的球数量，使其在整个规划过程中保持不变
persistent cachedCounts cachedNumJoints
if isempty(cachedCounts) || isempty(cachedNumJoints) || cachedNumJoints ~= nJoints
    % 首次调用（或机器人关节数变化）时，根据当前 X 确定每段的固定球数
    cachedCounts = zeros(nJoints,1);
    for k = 1:nJoints
        if k==1
            parent_joint_position = [0,0,0];
        else
            parent_joint_position = X(k-1, 1:3);
        end
        child_joint_position = X(k, 1:3);
        L = norm(child_joint_position - parent_joint_position);
        % 至少放置 2 个球，且按长度与半径估计需要的数量
        cachedCounts(k) = max(2, ceil(L / rad) + 1);
    end
    cachedNumJoints = nJoints;
end

center_cell = cell(nJoints,1);
radi_cell = cell(nJoints,1);

% 构造所有连杆的球心与半径（使用固定数量）
for k=1:nJoints
    if k==1
        parent_joint_position = [0,0,0];
    else
        parent_joint_position = X(k-1, 1:3);
    end
    child_joint_position = X(k, 1:3);

    nSpheres = cachedCounts(k);
    % 按固定数量在父—子关节之间做线性插值
    center_cell_k = arrayfun(@(x1, x2) linspace(x1, x2, nSpheres), ...
                             parent_joint_position', child_joint_position', 'UniformOutput', false);
    center_cell{k} = cell2mat(center_cell_k)';
    radi_cell{k} = rad * ones(size(center_cell{k},1), 1);
end

centers = cell2mat(center_cell);
radi = cell2mat(radi_cell);

end