% STOMP planner (Task5 override): uses soft EE orientation penalty in cost
% Copies Task4 helper; orientation handled by penalty in stompTrajCost

%Parameters
nDiscretize = 20; % number of discretized waypoint
nPaths = 20;     % number of sample paths
% Overrides from base workspace (RunTask5)
try
    if evalin('base','exist(''stomp_nDiscretize'',''var'')')
        nDiscretize = evalin('base','stomp_nDiscretize');
    end
    if evalin('base','exist(''stomp_nPaths'',''var'')')
        nPaths = evalin('base','stomp_nPaths');
    end
end
convergenceThreshold = 0.1; % convergence threshhold

% Initial guess of joint angles theta is just linear interpolation of q0 and qT
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

%% Precompute smoothing matrices
A_k = eye(nDiscretize - 1, nDiscretize - 1);
A = -2 * eye(nDiscretize, nDiscretize);
A(1:nDiscretize - 1, 2:nDiscretize) = A(1:nDiscretize - 1, 2:nDiscretize) + A_k;
A(2:nDiscretize, 1:nDiscretize - 1) = A(2:nDiscretize, 1:nDiscretize - 1) + A_k;
A = A(:, 2:end-1);
R = A' * A;
Rinv = inv(R);
M = 1 / nDiscretize * Rinv ./ max(Rinv, [], 1);
Rinv = 1.5*Rinv/sum(sum(Rinv));
Rinv_base = Rinv;
exploration_scale = 1.0;
try
    if evalin('base','exist(''stomp_exploration_scale'',''var'')')
        exploration_scale = evalin('base','stomp_exploration_scale');
    end
end
Rinv = exploration_scale * Rinv_base;

%% Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;

iter=0; maxIter = 50;
auto_rescue_attempted = false;
if ~exist('htext','var') || ~isgraphics(htext)
    htext = text(-0.2,0.6,0.7,['Iter 0/',num2str(maxIter)],'HorizontalAlignment','left','FontSize',14);
end
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    set(htext,'String',['Iter ',num2str(iter),'/',num2str(maxIter)]);
    QthetaOld = Qtheta;
    tic

    [theta_samples, em] = stompSamples(nPaths, Rinv, theta);
    Stheta = zeros(nPaths, nDiscretize);

    for k = 1:nPaths
        [S_k, ~] = stompTrajCost(robot_struct, theta_samples{k}, R, voxel_world);
        Stheta(k, :) = S_k;
    end

    S_valid = Stheta(:, 2:nDiscretize-1);

    % Boltzmann sampling
    J_theta_samples = sum(S_valid, 2);
    min_J = min(J_theta_samples);
    eta = 10;
    try
        if evalin('base','exist(''stomp_eta'',''var'')')
            eta = evalin('base','stomp_eta');
        end
    end
    scaled = -(J_theta_samples - min_J) / max(eta, eps);
    scaled = max(scaled, -700);
    exp_cost = exp(scaled);
    Z = sum(exp_cost);
    if Z <= 0 || ~isfinite(Z)
        P_k = ones(size(exp_cost)) / numel(exp_cost);
    else
        P_k = exp_cost / Z;
    end
    trajProb = repmat(P_k, 1, nDiscretize-2);

    % Gradient estimator and smoothing
    dtheta = stompDTheta(trajProb, em);
    dtheta_smoothed = zeros(numJoints, nDiscretize - 2);
    for m = 1:numJoints
        dtheta_smoothed(m, :) = Rinv * dtheta(m, :)';
    end

    theta_new = theta;
    theta_new(:, 2:nDiscretize-1) = theta(:, 2:nDiscretize-1) + dtheta_smoothed;
    theta = theta_new;

    % Orientation handled by soft penalty in stompTrajCost (no hard projection here)

    [~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
    toc

    Q_time = [Q_time Qtheta];
    RAR = 1/2 * sum(sum(theta(:, 2:nDiscretize-1) * R * theta(:, 2:nDiscretize-1)'));
    RAR_time = [RAR_time RAR];
    Qtheta
    RAR

    theta_animation{iter}=theta;

    if iter > maxIter
        disp('Maximum iteration (50) has reached.')
        break
    end

    if norm(dtheta_smoothed(:)) < 1e-12
        if ~auto_rescue_attempted
            disp('Estimated gradient is 0; increasing exploration noise and retrying...')
            exploration_scale = exploration_scale * 2;
            Rinv = exploration_scale * Rinv_base;
            auto_rescue_attempted = true;
            continue
        else
            disp('Estimated gradient is 0 and Theta is not updated: there could be no obstacle at all')
            break
        end
    end
end

disp('STOMP Finished.');
set(htext,'String',['Iter ',num2str(iter),'/',num2str(maxIter),' (done)']);

%% collision check
inCollision = false(nDiscretize, 1);
worldCollisionPairIdx = cell(nDiscretize,1);
for i = 1:nDiscretize
    [inCollision(i),sepDist] = checkCollision(robot,theta(:,i),world,"IgnoreSelfCollision","on","Exhaustive","on");
    [bodyIdx,worldCollisionObjIdx] = find(isnan(sepDist));
    worldCollidingPairs = [bodyIdx,worldCollisionObjIdx];
    worldCollisionPairIdx{i} = worldCollidingPairs;
end
isTrajectoryInCollision = any(inCollision)

%% optional videos and display (same as Task4)
enableVideoTraining = 1;
if enableVideoTraining == 1
    % Append scenario_id to training video filename if available
    scenario_suffix = '';
    try
        if evalin('base','exist(''scenario_id'',''var'')')
            scenario_suffix = ['_', num2str(evalin('base','scenario_id'))];
        end
    end
    v = VideoWriter(['KinvaGen3_Training', scenario_suffix, '.avi']);
    v.FrameRate = 15;
    open(v);
    theta_animation_tmp = theta_animation(~cellfun('isempty',theta_animation));
    nTraining = length(theta_animation_tmp);
    for k=0:5:nTraining
        set(htext,'String',['Iter ',num2str(k),'/',num2str(maxIter)]);
        theta_tmp = theta_animation_tmp{k+1};
        for t=1:size(theta_tmp,2)
            show(robot, theta_tmp(:,t),'PreservePlot', false, 'Frames', 'on');
            frame = getframe(gcf);
            writeVideo(v,frame);
        end
        pause(1/15);
    end
    close(v);
end

enableVideo = 0;
if enableVideo == 1
    v = VideoWriter('KinvaGen3_wEEConY3.avi');
    v.FrameRate =2;
    open(v);
    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        frame = getframe(gcf);
        writeVideo(v,frame);
        pause(5/20);
    end
    close(v);
end

displayAnimation = 1;
if displayAnimation
    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        pause(5/20);
    end
end

filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')