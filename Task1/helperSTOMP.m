%Parameters
% T = 5; 
nDiscretize = 20; % number of discretized waypoint
nPaths = 20; % number of sample paths
convergenceThreshold = 0.1; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of q0
% and qT
q0 = currentRobotJConfig;
qT = finalRobotJConfig;
numJoints = length(q0);
theta=zeros(numJoints, nDiscretize);
for k=1:length(q0)
    theta(k,:) = linspace(q0(k), qT(k), nDiscretize);
end

% by default, it loads the robot with the structure data format
robot_struct = loadrobot(robot_name); 

% store sampled paths
theta_samples = cell(1,nPaths);

%% for calculating the acceleration of theta
% Precompute
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1); 
R = A' * A;
Rinv = inv(R);
% The smoothing matrix M, normalized from Rinv by each column, no longer symmetric
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); 
% R inverse is normalized so that the exploration is controlled to have samples within the created voxel world
Rinv = 1.5*Rinv/sum(sum(Rinv)); 


%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;

iter=0;
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    % overall cost: Qtheta
    QthetaOld = Qtheta;
    % use tic and toc for printing out the running time
    tic

    [theta_samples, em] = stompSamples(nPaths, Rinv, theta); 
    Stheta = zeros(nPaths, nDiscretize);
    
    % Calculate cost for each sampled path k
    for k = 1:nPaths
        % stompTrajCost should return the cost for each time step i (S_i^k)
        % Note: The first and last waypoints are fixed, so we only care about time steps 2 to nDiscretize-1
        [S_k, ~] = stompTrajCost(robot_struct, theta_samples{k}, R, voxel_world);
        Stheta(k, :) = S_k;
    end
    
    % Only consider cost at movable waypoints (2 to nDiscretize-1)
    S_valid = Stheta(:, 2:nDiscretize-1); 

    %% TODO: Given the local traj cost, update local trajectory probability
    % 1. Compute Path Cost: J(theta^k) = sum(S_i^k) (excluding fixed start/end)
    J_theta_samples = sum(S_valid, 2);
    
    % 2. Find minimum cost and calculate exponential cost
    min_J = min(J_theta_samples);
    % Exponentiated Cost (Boltzmann distribution numerator): exp_cost_k = exp(-1/eta * (J(theta^k) - min_J))
    % eta (temperature/step-size) is typically an adjustable parameter. 
    % We will use a typical value like eta = 10 or 100 for normalization.
    eta = 10; 
    
    exp_cost = exp(-1/eta * (J_theta_samples - min_J));
    
    % 3. Calculate Local Trajectory Probability (P^k)
    % P^k = exp_cost_k / sum(exp_cost)
    P_k = exp_cost / sum(exp_cost);
    
    % 4. Calculate Expected Improvement (E_i^m)
    % E_i^m = P^k (local) probability for each waypoint i and joint m
    % This is usually done implicitly in the delta_theta calculation,
    % but we need the probability P_k repeated for each movable waypoint.
    
    % 5. Weight the noise by probability: The 'trajProb' is P_k repeated for each timestep
    % trajProb is (nPaths x (nDiscretize-2))
    trajProb = repmat(P_k, 1, nDiscretize-2);
    
    %% TODO: Compute delta theta (aka gradient estimator, the improvement of the delta)
    % dtheta: The raw improvement (gradient estimator)
    % em_valid: The noise terms for the movable waypoints (2 to nDiscretize-1)
    % em is a cell array (1 x numJoints), each cell is (nPaths x (nDiscretize-2))
    
    % The core formula is: delta_theta = Rinv * sum_k (P^k * epsilon^k)
    % We compute 'dtheta' (sum_k P^k * epsilon^k) first using the helper function.
    % dtheta: (numJoints x (nDiscretize-2))
    dtheta = stompDTheta(trajProb, em);
    
    % Apply smoothing (Rinv) to the raw gradient: delta_theta_raw = Rinv * dtheta
    % For each joint m: dtheta_smoothed_m = Rinv * dtheta_m
    dtheta_smoothed = zeros(numJoints, nDiscretize - 2);
    for m = 1:numJoints
        dtheta_smoothed(m, :) = Rinv * dtheta(m, :)';
    end

    theta_new = theta;
    theta_new(:, 2:nDiscretize-1) = theta(:, 2:nDiscretize-1) + dtheta_smoothed;
    theta = theta_new;
    
    [~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
 
    toc

    Q_time = [Q_time Qtheta];
    % control cost
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta % display overall cost
    RAR  % display control cost

    % Record the itermediate training trajectories for later animation
    theta_animation{iter}=theta;

    % Set the stopping iteration criteria:
    if iter > 50 
        disp('Maximum iteration (50) has reached.')
        break
    end

    if sum(dtheta_smoothed,'all') == 0
    disp('Estimated gradient is 0 and Theta is not updated: there could be no obstacle at all')
    break
    end

end

disp('STOMP Finished.');






%% check collision
inCollision = false(nDiscretize, 1); % initialization the collision status vector
worldCollisionPairIdx = cell(nDiscretize,1); % Initialization: Provide the bodies that are in collision

for i = 1:nDiscretize

    [inCollision(i),sepDist] = checkCollision(robot,theta(:,i),world,"IgnoreSelfCollision","on","Exhaustive","on");


    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist)); % Find collision pairs
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx];
    worldCollisionPairIdx{i} = worldCollidingPairs;

end
% Display whether there is collision:
isTrajectoryInCollision = any(inCollision)


%% Record the whole training/learning process in video file
%% ================== Video Recording (Stable, with obstacles & goal) ==================
% 控制开关
enableVideoTraining = 1;       % 训练过程中各个 iteration 的动画
enableVideoPlanned  = 0;       % 最终规划轨迹动画
displayAnimation    = 1;       % 只显示不保存
goalWorld = [0.35, 0.55, 0.35];   % 你的目标点世界坐标


% ========= 通用：创建一个稳定的绘图场景（共用） =========
function [fig, ax, hText, hObs, hGoal] = localSetupScene(figPos, voxel_world, goalWorld)
    fig = figure('Visible','on','Color','w','Position',figPos);
    ax  = axes('Parent',fig); axis(ax,'equal'); view(ax,3); grid(ax,'on');
    hold(ax,'on'); axis(ax,'manual');                    % 锁定坐标轴范围，防止 show 改动
    xlabel(ax,'X'); ylabel(ax,'Y'); zlabel(ax,'Z');

    xlim(ax, [-0.2 1.2]);
    ylim(ax, [-0.2 1.2]);
    zlim(ax, [0 0.7]);
    % === 固定为对角线的上方视角 ===
    axis(ax,'vis3d');                % 锁定比例，避免视角被缩放扭曲
    camproj(ax,'perspective');       % 透视投影更真实
    
    % 先锁定相机参数为手动（防止 show 重置）
    ax.CameraViewAngleMode = 'manual';
    ax.CameraPositionMode  = 'manual';
    ax.CameraTargetMode    = 'manual';
    ax.CameraUpVectorMode  = 'manual';
    
    % 用当前轴范围，自动计算对角线视角
    xl = xlim(ax); yl = ylim(ax); zl = zlim(ax);
    c  = [mean(xl) mean(yl) mean(zl)];      % 目标=场景中心
    sz = [diff(xl) diff(yl) diff(zl)];      % 场景尺寸
    
    margin = 0.35;                          % 离场景远一点，避免遮挡（可调 0.2~0.6）
    camtarget(ax, c);
    campos(ax, c + (1+margin)*[sz(1) sz(2) sz(3)]);  % 从对角线方向看向中心
    camup(ax, [0 0 1]);                     % z 轴向上
    
    % 设置方位角/俯仰角为对角线上方（再保险）
    view(ax, 30, 0);                       % az=45°, el=35°（可改 45~60）
    
    % 打光：让棕色障碍物更立体
    camlight(ax,'headlight'); 
    lighting(ax,'gouraud');



    % 文本叠加
    hText = text(ax, 0.02, 0.95, 0, 'Iteration = 0', ...
        'Units','normalized','Color','k','FontSize',14,'HorizontalAlignment','left');

    % ====== 绘制障碍物（若 voxel_world 可用则尝试 isosurface）======
    hObs = gobjects(1);
    try
        % 支持 sEDT 或二值占据栅格两种情况
        if isfield(voxel_world,'sEDT') && ~isempty(voxel_world.sEDT)
            sEDT   = voxel_world.sEDT;
            corner = voxel_world.Env_size(1,:);     % [xmin ymin zmin]
            voxel  = voxel_world.voxel_size;        % [vx vy vz]
            [Nx,Ny,Nz] = size(sEDT);
            [Xg,Yg,Zg] = ndgrid(0:Nx-1, 0:Ny-1, 0:Nz-1);
            Xw = corner(1) + Xg*voxel(1);
            Yw = corner(2) + Yg*voxel(2);
            Zw = corner(3) + Zg*voxel(3);

            p = patch(isosurface(Xw,Yw,Zw, sEDT, 0));           % 0 等值面近似障碍边界
            set(p,'Parent',ax,'FaceColor',[0.55 0.27 0.07],'EdgeColor','none','FaceAlpha',0.35);

            camlight(ax); lighting(ax,'gouraud');
            hObs = p;
        elseif isfield(voxel_world,'occ') && ~isempty(voxel_world.occ)
            occ    = voxel_world.occ;                % 逻辑/0-1 占据
            corner = voxel_world.Env_size(1,:);
            voxel  = voxel_world.voxel_size;
            [Nx,Ny,Nz] = size(occ);
            [Xg,Yg,Zg] = ndgrid(0:Nx-1, 0:Ny-1, 0:Nz-1);
            Xw = corner(1) + Xg*voxel(1);
            Yw = corner(2) + Yg*voxel(2);
            Zw = corner(3) + Zg*voxel(3);

            p = patch(isosurface(Xw,Yw,Zw, occ>0.5, 0.5));
            set(p,'Parent',ax,'FaceColor',[0.55 0.27 0.07],'EdgeColor','none','FaceAlpha',0.35);
            camlight(ax); lighting(ax,'gouraud');
            hObs = p;
        end
    catch
        % 如果体素信息不完整就忽略障碍物可视化
        hObs = gobjects(1);
    end

    % ====== 目标点（若提供）======
    hGoal = gobjects(1);
    if exist('goalWorld','var') && ~isempty(goalWorld) && numel(goalWorld)==3
        hGoal = plot3(ax, goalWorld(1), goalWorld(2), goalWorld(3), ...
    'o', 'MarkerSize', 5, 'MarkerFaceColor', [0.85 0.2 0.2], 'MarkerEdgeColor', 'none');

        % 小球更显眼的版本（取消注释使用）
        % [sx,sy,sz] = sphere(18);
        % rGoal = 0.03;
        % hGoal = surf(ax, rGoal*sx+goalWorld(1), rGoal*sy+goalWorld(2), rGoal*sz+goalWorld(3), ...
        %    'EdgeColor','none','FaceAlpha',0.9,'FaceColor',[0.9 0.3 0.3]);
    end
end

% ========= 通用：打开 VideoWriter（mp4 首选，失败回退 avi） =========
function v = localOpenWriter(fname, fps)
    try
        v = VideoWriter(fullfile(pwd, [fname '.mp4']), 'MPEG-4');
        v.Quality = 100;
    catch
        v = VideoWriter(fullfile(pwd, [fname '.avi']), 'Motion JPEG AVI');
        v.Quality = 100;
    end
    v.FrameRate = fps;
    open(v);
end

% ========= 训练过程视频：播放 theta_animation（只更新机器人）=========
if enableVideoTraining
    [figT, axT, hTextT, hObsT, hGoalT] = localSetupScene([100 100 1280 720], voxel_world, exist('goalWorld','var')*goalWorld);
    vT = localOpenWriter('KinovaGen3_Training', 5);

    theta_animation_tmp = {};
    if exist('theta_animation','var') && ~isempty(theta_animation)
        theta_animation_tmp = theta_animation(~cellfun('isempty', theta_animation));
    end
    nTraining = numel(theta_animation_tmp);

    for k = 1:5:nTraining
        if isgraphics(hObsT),  uistack(hObsT,'top');  end
        if isgraphics(hGoalT), uistack(hGoalT,'top'); end
        set(hTextT,'String',sprintf('Iteration = %d',k));

        theta_k = theta_animation_tmp{k};
        for t = 1:size(theta_k,2)
            show(robot, theta_k(:,t), 'Parent', axT, 'PreservePlot', false, 'Frames', 'off');
            show(robot, theta_k(:,t), 'Parent', axT, 'PreservePlot', false, 'Frames', 'off');


            hold(axT,'on');                               % 某些版本 show 会重置 hold
            drawnow limitrate nocallbacks;
            frame = getframe(figT);
            writeVideo(vT, frame);
        end
    end
    close(vT);
end

% ========= 规划轨迹视频：播放最终 theta （只更新机器人）=========
if enableVideoPlanned
    [figP, axP, ~, hObsP, hGoalP] = localSetupScene([120 120 1280 720], voxel_world, exist('goalWorld','var')*goalWorld);
    vP = localOpenWriter('KinovaGen3_Planned', 2);

    for t = 1:size(theta,2)
        if isgraphics(hObsP),  uistack(hObsP,'top');  end
        if isgraphics(hGoalP), uistack(hGoalP,'top'); end

        show(robot, theta(:,t), 'Parent', axP, 'PreservePlot', false, 'Frames', 'off');
        hold(axP,'on');
        drawnow limitrate nocallbacks;
        frame = getframe(figP);
        writeVideo(vP, frame);
        % pause(0.25);   % 纯为人眼预览，与写文件无关
    end
    close(vP);
end

% ========= 只预览（不保存）=========
if displayAnimation
    [figD, axD, ~, hObsD, hGoalD] = localSetupScene([140 140 1280 720], voxel_world, exist('goalWorld','var')*goalWorld);
    for t = 1:size(theta,2)
        if isgraphics(hObsD),  uistack(hObsD,'top');  end
        if isgraphics(hGoalD), uistack(hGoalD,'top'); end

        show(robot, theta(:,t), 'Parent', axD, 'PreservePlot', false, 'Frames', 'off');
        hold(axD,'on');
        drawnow;
        % pause(0.25);
    end
end
%% ================== End Video Recording ==================
