% Task5 compare runner: show path difference before/after adding EE orientation constraint
% This script runs STOMP twice under the same obstacle scenario:
% 1) Without orientation soft penalty (baseline)
% 2) With orientation soft penalty enabled (keep EE axis upright)
% Then overlays the end-effector (EE) Cartesian paths for visual comparison.

%% Add paths — ensure Task5 overrides are found first
addpath(genpath('Task5'));              % local overrides (this runner)
addpath(genpath('Task4'));              % reuse Task4 implementations

%% STOMP safety and resolution (reuse recommended values from RunTask5)
stomp_safety_margin = 0.07;
stomp_alpha = 300;
stomp_sphere_radius = 0.04;
voxel_size_override = [0.015 0.015 0.015];

%% STOMP sampling and discretization
stomp_nDiscretize = 30;
stomp_nPaths = 40;
stomp_eta = 8;

%% Task 5: orientation constraint parameters (will be toggled below)
task5_enable = true;             % keep flag on; toggle weight to enable/disable
task5_keep_axis = 'y';           % keep end-effector y-axis aligned
task5_world_axis = [0;0;1];      % align with world Z direction
task5_penalty_norm = 'l1';       % penalty norm ('l1' or 'l2') for angle metric
task5_penalty_metric = 'angle_hinge'; % enforce max angle with hinge penalty
task5_angle_threshold_deg = 8;   % allow up to ~8 degrees deviation without penalty

%% Robot setup
robot_name = 'kinovaGen3';
robot = loadrobot(robot_name,'DataFormat','column');
numJoints = numel(robot.homeConfiguration);

%% Start and goal
currentRobotJConfig = robot.homeConfiguration;

% Choose scenario (same as RunTask5)
scenario_id = 3;  % 可切换为 1/2/3/4
switch scenario_id
    case 1  % 斜墙阻断：目标放在右上，需要绕墙走角度
        goalPos = [0.35, 0.40, 0.30];
        goalPos = [0.60, 0.46, 0.44];
    case 3  % 组合场景（case1 + case2）：斜墙 + 低斜板 + 上方横梁
        goalPos = [0.35, 0.30, 0.3];  % 合理挑战性的终点，需绕墙同时处理高度
    case 4  % 交错砖块：穿越交错小块
        goalPos = [0.70, 0.44, 0.36];
    otherwise
        goalPos = [0.50, 0.32, 0.35];
end

% Final pose: align EE selected axis to world target axis at goal
try
    world_dir_goal = task5_world_axis(:) / norm(task5_world_axis);
catch
    world_dir_goal = [0;0;1];
end
up_ref_goal = [0;1;0];
if abs(dot(up_ref_goal, world_dir_goal)) > 0.99
    up_ref_goal = [1;0;0];
end
axg = lower(string(task5_keep_axis));
switch axg
    case "x"
        xg = world_dir_goal;
        zg = cross(xg, up_ref_goal); if norm(zg) < eps, up_ref_goal = [1;0;0]; zg = cross(xg, up_ref_goal); end
        zg = zg / norm(zg);
        yg = cross(zg, xg); yg = yg / norm(yg);
        R_goal = [xg, yg, zg];
    case "y"
        yg = world_dir_goal;
        xg = cross(up_ref_goal, yg); if norm(xg) < eps, up_ref_goal = [1;0;0]; xg = cross(up_ref_goal, yg); end
        xg = xg / norm(xg);
        zg = cross(xg, yg); zg = zg / norm(zg);
        R_goal = [xg, yg, zg];
    otherwise % "z"
        zg = world_dir_goal;
        xg = cross(up_ref_goal, zg); if norm(xg) < eps, up_ref_goal = [1;0;0]; xg = cross(up_ref_goal, zg); end
        xg = xg / norm(xg);
        yg = cross(zg, xg); yg = yg / norm(yg);
        R_goal = [xg, yg, zg];
end
tformFinal = trvec2tform(goalPos) * rotm2tform(R_goal);
poseFinal = goalPos;

% Inverse kinematics for final joint config
ik = inverseKinematics('RigidBodyTree', robot);
% Tighten IK solver to better hit the desired position
ik.SolverParameters.MaxIterations = 1000;
ik.SolverParameters.GradientTolerance = 1e-8;
ik.SolverParameters.StepTolerance = 1e-8;
ik.SolverParameters.AllowRandomRestart = true;
% Lock both position & orientation at goal (consistent alignment)
weights = [1 1 1 1 1 1];
initialguess = currentRobotJConfig;
eeName = robot.BodyNames{end};
[finalRobotJConfig, ~] = ik(eeName, tformFinal, weights, initialguess);

% Align start EE axis to desired world direction and override starting joint config
try
    world_dir = task5_world_axis(:) / norm(task5_world_axis);
catch
    world_dir = [0;0;1];
end
T_home = getTransform(robot, robot.homeConfiguration, eeName);
startPos = tform2trvec(T_home)';
% Choose a reference up vector not collinear with target
up_ref = [0;1;0];
if abs(dot(up_ref, world_dir)) > 0.99
    up_ref = [1;0;0];
end

ax = lower(string(task5_keep_axis));
switch ax
    case "x"
        x_axis = world_dir;
        z_axis = cross(x_axis, up_ref); if norm(z_axis) < eps, up_ref = [1;0;0]; z_axis = cross(x_axis, up_ref); end
        z_axis = z_axis / norm(z_axis);
        y_axis = cross(z_axis, x_axis); y_axis = y_axis / norm(y_axis);
        R_start = [x_axis, y_axis, z_axis];
    case "y"
        y_axis = world_dir;
        x_axis = cross(up_ref, y_axis); if norm(x_axis) < eps, up_ref = [1;0;0]; x_axis = cross(up_ref, y_axis); end
        x_axis = x_axis / norm(x_axis);
        z_axis = cross(x_axis, y_axis); z_axis = z_axis / norm(z_axis);
        R_start = [x_axis, y_axis, z_axis];
    otherwise % "z"
        z_axis = world_dir;
        x_axis = cross(up_ref, z_axis); if norm(x_axis) < eps, up_ref = [1;0;0]; x_axis = cross(up_ref, z_axis); end
        x_axis = x_axis / norm(x_axis);
        y_axis = cross(z_axis, x_axis); y_axis = y_axis / norm(y_axis);
        R_start = [x_axis, y_axis, z_axis];
end

tformStart = trvec2tform(startPos') * rotm2tform(R_start);
[startRobotJConfig, ~] = ik(eeName, tformStart, weights, robot.homeConfiguration);
currentRobotJConfig = startRobotJConfig;

%% Build obstacles via Task4 helper — only use scenario_id
helperCreateObstaclesTask4;

%% Goal free-space check and correction (reuse logic)
% Allow user to strictly keep the original goal position without auto-correction
if ~exist('task5_strict_goal_lock','var'), task5_strict_goal_lock = true; end
if ~task5_strict_goal_lock
try
    vx = voxel_world.voxel_size(1); vy = voxel_world.voxel_size(2); vz = voxel_world.voxel_size(3);
    xmin = voxel_world.Env_size(1,1); ymin = voxel_world.Env_size(1,2); zmin = voxel_world.Env_size(1,3);
    xi = max(1, min(size(voxel_world.sEDT,1), round((goalPos(1) - xmin)/vx)));
    yi = max(1, min(size(voxel_world.sEDT,2), round((goalPos(2) - ymin)/vy)));
    zi = max(1, min(size(voxel_world.sEDT,3), round((goalPos(3) - zmin)/vz)));
    sedtVal = voxel_world.sEDT(xi, yi, zi);
    if sedtVal <= 0
        step = max([vx, vy, vz]) * 2;
        maxR = 0.25;
        dirs = [0 0 1; 0 1 0; 0 -1 0; 1 0 0; -1 0 0; 1 1 0; -1 1 0; 1 -1 0; -1 -1 0; 0 1 1; 0 -1 1; 1 0 1; -1 0 1];
        found = false;
        for r = step:step:maxR
            for di = 1:size(dirs,1)
                cand = goalPos + r * dirs(di,:);
                cand = max([xmin ymin zmin], min([xmin xmin xmin] + voxel_world.Env_size(2,:), cand));
                xi = max(1, min(size(voxel_world.sEDT,1), round((cand(1) - xmin)/vx)));
                yi = max(1, min(size(voxel_world.sEDT,2), round((cand(2) - ymin)/vy)));
                zi = max(1, min(size(voxel_world.sEDT,3), round((cand(3) - zmin)/vz)));
                if voxel_world.sEDT(xi, yi, zi) > 0
                    goalPos = cand; found = true; break;
                end
            end
            if found, break; end
        end
        if ~found
            goalPos(3) = goalPos(3) + 0.10;
        end
        tformFinal = trvec2tform(goalPos) * goalOrient;
        [finalRobotJConfig, ~] = ik(eeName, tformFinal, weights, initialguess);
        poseFinal = goalPos;
    end
catch ME
    warning('目标位置自由空间校验失败：%s', ME.message);
end
else
    disp('已启用严格终点锁定：不进行目标位置的自由空间修正。');
end

%% Visualize once
x0 = currentRobotJConfig;
viz_axis_limits = [-1.2 1.2 -1.0 1.0 -0.5 1.2];
viz_view = [140 25];
viz_axis_equal = true;
viz_fig_position = [200 200 900 700];
viz_frames_on = true;
helperInitialVisualizerKINOVA;

%% Run 1: without orientation penalty (baseline)
task5_penalty_weight = 0;   % disable soft penalty
helperSTOMP;
theta_before = theta;       % capture trajectory

% EE path for run 1
pts_before = zeros(3, size(theta_before,2));
for t = 1:size(theta_before,2)
    T = getTransform(robot, theta_before(:,t), eeName);
    pts_before(:,t) = tform2trvec(T)';
end

%% Run 2: with orientation penalty (constrained)
task5_penalty_weight = 900; % stronger enforcement beyond threshold
helperSTOMP;
theta_after = theta;        % capture trajectory

% EE path for run 2
pts_after = zeros(3, size(theta_after,2));
for t = 1:size(theta_after,2)
    T = getTransform(robot, theta_after(:,t), eeName);
    pts_after(:,t) = tform2trvec(T)';
end

% Report final distances to goal position for both runs
dist_before = norm(pts_before(:,end) - goalPos(:));
dist_after  = norm(pts_after(:,end)  - goalPos(:));
fprintf('末端到目标位置距离(无约束): %.3f cm\n', dist_before*100);
fprintf('末端到目标位置距离(有姿态约束): %.3f cm\n', dist_after*100);
if dist_after > 0.01
    warning('末端未到达目标位置(>1 cm): 当前距离 %.2f cm', dist_after*100);
end

%% Overlay EE paths for comparison
hold on;
% Mark the desired goal position explicitly
h_goal = scatter3(goalPos(1), goalPos(2), goalPos(3), 60, 'r', 'filled', 'DisplayName','Goal position');
% Brightly mark the two axes (EE selected axis and world target axis) at multiple timesteps
axis_len = 0.12; % arrow length in meters
% Determine EE axis index from keep_axis
ax_sel = lower(string(task5_keep_axis));
switch ax_sel
    case "x", ax_idx = 1; ee_color = [1 0 0]; ee_label = 'EE x-axis (samples)';
    case "y", ax_idx = 2; ee_color = [0 1 1]; ee_label = 'EE y-axis (samples)';
    otherwise, ax_idx = 3; ee_color = [0 0 1]; ee_label = 'EE z-axis (samples)';
end
% World target axis and color mapping
try
    world_dir = task5_world_axis(:) / norm(task5_world_axis);
catch
    world_dir = [0;0;1];
end
if abs(dot(world_dir, [1;0;0])) > 0.99
    world_color = [1 0 0]; world_label = 'World x-axis';
elseif abs(dot(world_dir, [0;1;0])) > 0.99
    world_color = [0 1 0]; world_label = 'World y-axis';
elseif abs(dot(world_dir, [0;0;1])) > 0.99
    world_color = [0 0 1]; world_label = 'World z-axis';
else
    world_color = [1 1 0]; world_label = 'World axis';
end
% Sample timesteps along the constrained trajectory and draw arrows
N_after = size(theta_after,2);
sample_count = min(8, max(1, N_after - 2));
idxs = unique(round(linspace(1, N_after, sample_count))); % include start/end
% First arrows with DisplayName for legend
Tfirst = getTransform(robot, theta_after(:,idxs(1)), eeName);
Rfirst = Tfirst(1:3,1:3);
pos_first = tform2trvec(Tfirst)';
ee_axis_first = Rfirst(:,ax_idx); if norm(ee_axis_first) > eps, ee_axis_first = ee_axis_first / norm(ee_axis_first); end
h_ee = quiver3(pos_first(1),pos_first(2),pos_first(3), ee_axis_first(1)*axis_len, ee_axis_first(2)*axis_len, ee_axis_first(3)*axis_len, 0, 'Color', ee_color, 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', ee_label);
h_world = quiver3(pos_first(1),pos_first(2),pos_first(3), world_dir(1)*axis_len, world_dir(2)*axis_len, world_dir(3)*axis_len, 0, 'Color', world_color, 'LineWidth', 2, 'MaxHeadSize', 2, 'DisplayName', world_label);
% Remaining arrows hidden from legend to avoid clutter
for k = 2:numel(idxs)
    Tt = getTransform(robot, theta_after(:,idxs(k)), eeName);
    Rt = Tt(1:3,1:3);
    pos_t = tform2trvec(Tt)';
    ee_axis_t = Rt(:,ax_idx); if norm(ee_axis_t) > eps, ee_axis_t = ee_axis_t / norm(ee_axis_t); end
    quiver3(pos_t(1),pos_t(2),pos_t(3), ee_axis_t(1)*axis_len, ee_axis_t(2)*axis_len, ee_axis_t(3)*axis_len, 0, 'Color', ee_color, 'LineWidth', 2, 'MaxHeadSize', 2, 'HandleVisibility','off');
    quiver3(pos_t(1),pos_t(2),pos_t(3), world_dir(1)*axis_len, world_dir(2)*axis_len, world_dir(3)*axis_len, 0, 'Color', world_color, 'LineWidth', 2, 'MaxHeadSize', 2, 'HandleVisibility','off');
end

hb = plot3(pts_before(1,:), pts_before(2,:), pts_before(3,:), 'r-', 'LineWidth', 2);
%scatter3(pts_before(1,2:end-1), pts_before(2,2:end-1), pts_before(3,2:end-1), 22, 'r', 'filled');
ha = plot3(pts_after(1,:),  pts_after(2,:),  pts_after(3,:),  'b-', 'LineWidth', 2);
%scatter3(pts_after(1,2:end-1),  pts_after(2,2:end-1),  pts_after(3,2:end-1),  22, 'b', 'filled');
legend([hb, ha, h_ee, h_world, h_goal], {'Unconstrained EE path','Constrained EE path',ee_label,world_label,'Goal position'}, 'Location', 'best');
title('Task5: Path differences before/after adding EE orientation constraint');
hold off;

%% Save both trajectories for reproducibility
save('Theta_Task5_Compare_Before.mat','theta_before');
save('Theta_Task5_Compare_After.mat','theta_after');

disp('Task5 对比完成：已生成两条轨迹并叠加绘制。');