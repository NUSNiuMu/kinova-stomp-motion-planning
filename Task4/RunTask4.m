% Task4 runner: custom scenario with multiple obstacles and STOMP planning
% Uses code under EE5112_Project_2_Collision-Avoidance-Code_Incomplete

% Add paths (use only Task4 local implementations)
addpath(genpath('Task4'));   % custom obstacles and local STOMP for Task4

% 推荐：更保守的避障与更精细体素（可开箱即用）
stomp_safety_margin = 0.07;      % 安全余量（m），默认 0.05
stomp_alpha = 300;               % 代价强度，默认 200
stomp_sphere_radius = 0.04;      % 机器人球近似半径，默认 0.05
voxel_size_override = [0.01 0.01 0.01]; % 体素分辨率（m），默认 0.02

% STOMP 参数（提高采样与离散密度）
stomp_nDiscretize = 30;          % 默认 20
stomp_nPaths = 40;               % 默认 20
stomp_eta = 8;                   % 默认 10（温度/步长）

% Robot setup
robot_name = 'kinovaGen3';
robot = loadrobot(robot_name,'DataFormat','column');
numJoints = numel(robot.homeConfiguration);

% Start and goal selection
currentRobotJConfig = robot.homeConfiguration; % start at home

% 选择场景并设置与之冲突的目标位姿（直线必碰撞）
scenario_id = 3;  % 可切换为 1/2/3/4
switch scenario_id
    case 1  % 斜墙阻断：目标放在右上，需要绕墙走角度
        goalPos = [0.35, 0.40, 0.30];
    case 2  % 高低障碍：需先低再高或相反的高度变化
        goalPos = [0.60, 0.46, 0.44];
    case 3  % 组合场景（case1 + case2）：斜墙 + 低斜板 + 上方横梁
        goalPos = [0.35, 0.30, 0.3];  % 合理挑战性的终点，需绕墙同时处理高度
    case 4  % 交错砖块：穿越交错小块
        goalPos = [0.70, 0.44, 0.36];
    otherwise
        goalPos = [0.50, 0.32, 0.35];
end
goalOrient = axang2tform([0 0 1 0]);
tformFinal = trvec2tform(goalPos) * goalOrient;
poseFinal = goalPos; % for visualizer dot

% Compute IK for final config
ik = inverseKinematics('RigidBodyTree', robot);
weights = [0.25 0.25 0.25 1 1 1];
initialguess = currentRobotJConfig;
eeName = robot.BodyNames{end};
[finalRobotJConfig, ~] = ik(eeName, tformFinal, weights, initialguess);

% Build Task4 obstacles and voxel world
% 选择场景类型（当使用 scenario_id 时将被覆盖）
scene_type = 'corridor';            % 使用自定义 4 场景时此项仅作占位
% Corridor controls (applies to both scenes) — easier settings
corridor_gap_y = 0.3;             % 通道缝隙宽度（米）：加宽以降低难度
corridor_wall_thick_y = 0.02;      % 墙厚（米）：变薄以降低难度

% Zigzag-specific parameters (segment count/positions and beam/pillar)
% Segments: rows of [x, y_center, z]
zigzag_segments = [
    0.45, 0.34, 0.30;
    0.55, 0.42, 0.30;
    0.65, 0.34, 0.30
];
% Override wall length/height for zigzag if needed
zigzag_wall_len_x = 0.30;          % 每段墙沿 x 方向长度
zigzag_wall_height_z = 0.25;       % 墙高度（z 方向）

% Pillar settings — disabled to reduce difficulty
zigzag_pillar_enable = false;
zigzag_pillar_center = [0.58, 0.38, 0.30];
zigzag_pillar_dims   = [0.08, 0.08, 0.25];

% Beam settings — disabled to reduce difficulty
zigzag_beam_enable = false;
zigzag_beam_center = [0.58, 0.38, 0.45];
zigzag_beam_dims   = [0.25, 0.25, 0.05];
% 传入自定义场景编号，构建非轴对齐障碍
helperCreateObstaclesTask4; % 生成 world 和 voxel_world

% 目标自由空间校验与修正（终点不能在障碍物中）
try
    vx = voxel_world.voxel_size(1); vy = voxel_world.voxel_size(2); vz = voxel_world.voxel_size(3);
    xmin = voxel_world.Env_size(1,1); ymin = voxel_world.Env_size(1,2); zmin = voxel_world.Env_size(1,3);
    % 查询 sEDT 值（<=0 表示处于障碍或其边界内部）
    xi = max(1, min(size(voxel_world.sEDT,1), round((goalPos(1) - xmin)/vx)));
    yi = max(1, min(size(voxel_world.sEDT,2), round((goalPos(2) - ymin)/vy)));
    zi = max(1, min(size(voxel_world.sEDT,3), round((goalPos(3) - zmin)/vz)));
    sedtVal = voxel_world.sEDT(xi, yi, zi);
    if sedtVal <= 0
        step = max([vx, vy, vz]) * 2;
        maxR = 0.25;  % 最大搜索半径
        dirs = [
            0 0 1;
            0 1 0; 0 -1 0; 1 0 0; -1 0 0;
            1 1 0; -1 1 0; 1 -1 0; -1 -1 0;
            0 1 1; 0 -1 1; 1 0 1; -1 0 1;
        ];
        found = false;
        for r = step:step:maxR
            for di = 1:size(dirs,1)
                cand = goalPos + r * dirs(di,:);
                % 环境边界裁剪
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
            goalPos(3) = goalPos(3) + 0.10; % 兜底：抬高 z
        end
        % 依据修正后的目标重新计算 IK
        tformFinal = trvec2tform(goalPos) * goalOrient;
        [finalRobotJConfig, ~] = ik(eeName, tformFinal, weights, initialguess);
        poseFinal = goalPos;
    end
catch ME
    warning('目标位置自由空间校验失败：%s', ME.message);
end

% Initial visualization (Task1 helper)
x0 = currentRobotJConfig; % helper expects x0 and numJoints
% Visualization parameters override
viz_axis_limits = [-1.2 1.2 -1.0 1.0 -0.5 1.2];
viz_view = [140 25];
viz_axis_equal = true;
viz_fig_position = [200 200 900 700];
viz_frames_on = true;
helperInitialVisualizerKINOVA;

% Run STOMP planning (Task1 helper reads variables from base workspace)
helperSTOMP;

% 叠加可视化：绘制末端执行器路径轨迹与中间点
try
    eeName = robot.BodyNames{end};
    % 最终轨迹点
    pts = zeros(3, size(theta,2));
    for t = 1:size(theta,2)
        T = getTransform(robot, theta(:,t), eeName);
        pts(:,t) = tform2trvec(T)';
    end
    hold on;
    plot3(pts(1,:), pts(2,:), pts(3,:), 'm-', 'LineWidth', 2); % 终轨迹
    scatter3(pts(1,2:end-1), pts(2,2:end-1), pts(3,2:end-1), 24, 'm', 'filled'); % 中间点

    % 选取若干迭代的中间构型点，展示避障演化
    if exist('theta_animation','var') && ~isempty(theta_animation)
        pick = unique(round(linspace(1, numel(theta_animation), min(3, numel(theta_animation)))));
        for p = pick
            th = theta_animation{p};
            eePts = zeros(3, size(th,2));
            for tt = 1:size(th,2)
                Tt = getTransform(robot, th(:,tt), eeName);
                eePts(:,tt) = tform2trvec(Tt)';
            end
            plot3(eePts(1,:), eePts(2,:), eePts(3,:), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
        end
    end
    hold off;
catch ME
    warning('路径叠加可视化失败：%s', ME.message);
end