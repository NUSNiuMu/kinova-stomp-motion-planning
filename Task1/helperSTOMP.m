%Parameters
% T = 5; 
nDiscretize = 20; % number of discretized waypoint
nPaths = 30; % number of sample paths
convergenceThreshold = 0.05; % convergence threshhold

disp('>>> helperSTOMP: INIT — building initial theta from q0 to qT');

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
disp('>>> helperSTOMP: PRECOMPUTE — building A, R, Rinv, M');
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1); 
% --- 新增：一阶差分矩阵（速度正则），覆盖到末段 ---
m = nDiscretize - 2;                 % 内点维度，与 A(:,2:end-1) 一致

% 一阶差分矩阵（速度正则）：(m-1) x m
% 每行是 e_{i+1} - e_i，确保 D'*D 为 m x m，与 A'*A 尺寸匹配
D = diff(eye(m), 1, 1);              % 等价：spdiags([-ones(m-1,1) ones(m-1,1)],[0 1], m-1, m)

lambda_v = 0.1;                      % 速度正则权重，0.05~0.3 之间调
R = (A' * A) + lambda_v * (D' * D);  % 尺寸一致：m x m
Rinv = inv(R);
% The smoothing matrix M, normalized from Rinv by each column, no longer symmetric
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1); 
% R inverse is normalized so that the exploration is controlled to have samples within the created voxel world
Rinv = 1.5*Rinv/sum(sum(Rinv)); 

disp('>>> helperSTOMP: COST0 — evaluating initial Qtheta');
%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

% ---- 改为：
[~, Q_state] = stompTrajCost(robot_struct, theta, R, voxel_world);            % 仅 ∑ q(θ_i)
RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));  % ½ θᵀRθ（内点）
Q_total = Q_state + RAR;                                                      % 轨迹总成本
Q_total_old = inf;                                                            % 用总成本做收敛


iter=0;
disp('>>> helperSTOMP: ENTER LOOP — starting STOMP optimization (no per-iter prints by request)');
while abs(Q_total - Q_total_old) > convergenceThreshold
    iter=iter+1;
    % overall cost: Qtheta
    Q_total_old = Q_total;
    % use tic and toc for printing out the running time
    tic
    %% TODO: Complete the following code. The needed functions are already given or partially given in the folder.
    %% TODO: Sample noisy trajectories (only inner knots)
    [theta_samples, em] = stompSamples(nPaths, Rinv, theta);   % sigma=Rinv per PPT
    
    %% TODO: Calculate Local trajectory cost for each sampled trajectory (per-time-step)
    % variable declaration (holder for the cost):
    Stheta = zeros(nPaths, nDiscretize);
    for k = 1:nPaths
        [S_k, ~] = stompTrajCost(robot_struct, theta_samples{k}, R, voxel_world);
        Stheta(k, :) = S_k;
    end
    
    %% TODO: Given the local traj cost, update local trajectory probability (softmin, per time-step)
    trajProb = zeros(nPaths, nDiscretize);
    for t = 1:nDiscretize
        c = Stheta(:, t);
        c = c - min(c);              % shift to avoid overflow
        s = std(c) + eps;            % temperature-like scale per PPT
        w = exp(-c / s);             % softmin
        trajProb(:, t) = w / (sum(w) + eps);  % normalization per column (time-step)
    end
    
    %% TODO: Compute delta theta (aka gradient estimator, the improvement of the delta)
    % unbiased weighting (baseline) per PPT
    dtheta = stompDTheta(trajProb, em);
    
    % Smooth & update (only inner knots), step size eta
    dtheta_smoothed = zeros(size(theta));
    dtheta_smoothed(:, 2:nDiscretize-1) = ( dtheta(:, 2:nDiscretize-1) * M );
    theta(:, 2:nDiscretize-1) = theta(:, 2:nDiscretize-1) + dtheta_smoothed(:, 2:nDiscretize-1);
    
    %% TODO: Compute the cost of the new trajectory
    % 重新计算：状态项 + 平滑项 + 总成本
    [~, Q_state] = stompTrajCost(robot_struct, theta, R, voxel_world);                 % 仅 ∑ q(θ_i)
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));  % ½ θᵀRθ
    Q_total_new = Q_state + RAR;
    
    % 记录/打印（你原有的数组名沿用）
    Q_time    = [Q_time, Q_total_new];   % 建议用总成本作收敛曲线
    RAR_time  = [RAR_time, RAR];
    
    Q_total   = Q_total_new;             % 更新当前总成本
    Q_total                                % 显示总成本（和你原来风格一致）
    Q_state                                % 可选：显示状态项 ∑q(θ_i)
    RAR                                     % 显示平滑项 ½θᵀRθ


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
disp('>>> helperSTOMP: LOOP FINISHED');

disp('STOMP Finished.');

%% check collision
disp('>>> helperSTOMP: COLLISION CHECK — running checkCollision over the planned trajectory');
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
disp(['>>> helperSTOMP: COLLISION CHECK DONE — any collision? ', mat2str(isTrajectoryInCollision)]);

%% Record the whole training/learning process in video file
enableVideoTraining = 1;

disp('>>> helperSTOMP: TRAINING VIDEO — begin');
v = VideoWriter('KinvaGen3_Training.avi');
v.FrameRate = 15;
open(v);

htext = text(-0.2,0.6,0.7,'Iteration = 0','HorizontalAlignment','left','FontSize',14);

if enableVideoTraining == 1
    theta_animation_tmp = theta_animation(~cellfun('isempty',theta_animation));
    nTraining = length(theta_animation_tmp);
    for k=0:5:nTraining
        
        UpdatedText = ['Iteration = ',num2str(k)];
        set(htext,'String',UpdatedText)
        theta_tmp = theta_animation_tmp{k+1};

        for t=1:size(theta_tmp,2)
            show(robot, theta_tmp(:,t),'PreservePlot', false, 'Frames', 'on');
            %             drawnow;
            frame = getframe(gcf);
            writeVideo(v,frame);
%             pause(1/15);
            %     pause;
        end
        pause(1/15);
    end
end
close(v);
disp('>>> helperSTOMP: TRAINING VIDEO — saved KinvaGen3_Training.avi');

%% Record planned trajectory to video files
enableVideo = 1;
if enableVideo == 1
    disp('>>> helperSTOMP: TRAJ VIDEO — begin');
    v = VideoWriter('KinvaGen3_wEEConY3.avi');
    v.FrameRate =2;
    open(v);

    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        frame = getframe(gcf);
        writeVideo(v,frame);
        pause(5/20);
        %     pause;
    end
    close(v);
    disp('>>> helperSTOMP: TRAJ VIDEO — saved KinvaGen3_wEEConY3.avi');
end

%% Show the planned trajectory
displayAnimation = 1;
if displayAnimation
    disp('>>> helperSTOMP: DISPLAY — showing planned trajectory');
    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        pause(5/20);
        %     pause;
    end
    disp('>>> helperSTOMP: DISPLAY — done');
end

%% save data
disp('>>> helperSTOMP: SAVE — writing theta to MAT file');
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')
disp(['>>> helperSTOMP: SAVE — saved ', filename]);
