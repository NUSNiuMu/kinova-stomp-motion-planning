% STOMP planner (Task4 local copy, independent of Task1 path)
%Parameters
% T = 5; 
nDiscretize = 20; % number of discretized waypoint
nPaths = 20; % number of sample paths
% Overrides from base workspace (RunTask4)
try
    if evalin('base','exist(''stomp_nDiscretize'',''var'')')
        nDiscretize = evalin('base','stomp_nDiscretize');
    end
    if evalin('base','exist(''stomp_nPaths'',''var'')')
        nPaths = evalin('base','stomp_nPaths');
    end
end
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
% Base copy for scaling exploration
Rinv_base = Rinv;
% Exploration scale override from base workspace
exploration_scale = 1.0;
try
    if evalin('base','exist(''stomp_exploration_scale'',''var'')')
        exploration_scale = evalin('base','stomp_exploration_scale');
    end
end
Rinv = exploration_scale * Rinv_base;


%%
%Planner
Q_time = [];   % Trajectory cost Q(theta), t-vector
RAR_time = [];

[~, Qtheta] = stompTrajCost(robot_struct, theta, R, voxel_world);
QthetaOld = 0;

iter=0; maxIter = 50;
auto_rescue_attempted = false;
% Live iteration text on figure (created by visualizer)
if ~exist('htext','var') || ~isgraphics(htext)
    htext = text(-0.2,0.6,0.7,['Iter 0/',num2str(maxIter)],'HorizontalAlignment','left','FontSize',14);
end
while abs(Qtheta - QthetaOld) > convergenceThreshold
    iter=iter+1;
    % update iteration display
    set(htext,'String',['Iter ',num2str(iter),'/',num2str(maxIter)]);
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
    try
        if evalin('base','exist(''stomp_eta'',''var'')')
            eta = evalin('base','stomp_eta');
        end
    end
    
    % numerical stabilization to avoid underflow
    scaled = -(J_theta_samples - min_J) / max(eta, eps);
    scaled = max(scaled, -700); % clip to avoid exp(-Inf)
    exp_cost = exp(scaled);
    
    % 3. Calculate Local Trajectory Probability (P^k)
    % P^k = exp_cost_k / sum(exp_cost)
    Z = sum(exp_cost);
    if Z <= 0 || ~isfinite(Z)
        P_k = ones(size(exp_cost)) / numel(exp_cost); % fallback to uniform when degenerate
    else
        P_k = exp_cost / Z;
    end
    
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
% Finalize iteration display
set(htext,'String',['Iter ',num2str(iter),'/',num2str(maxIter),' (done)']);


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
    % reuse htext created earlier
    theta_animation_tmp = theta_animation(~cellfun('isempty',theta_animation));
    nTraining = length(theta_animation_tmp);
    for k=0:5:nTraining
        % overlay iteration progress in training video
        set(htext,'String',['Iter ',num2str(k),'/',num2str(maxIter)]);
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
    close(v);
end


%% Record planned trajectory to video files
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
        %     pause;
    end
    close(v);
end
%% Show the planned trajectory
displayAnimation = 1;
if displayAnimation
    for t=1:size(theta,2)
        show(robot, theta(:,t),'PreservePlot', false, 'Frames', 'on');
        drawnow;
        pause(5/20);
        %     pause;
    end
end


%% save data
filename = ['Theta_nDisc', num2str(nDiscretize),'_nPaths_', num2str(nPaths), '.mat'];
save(filename,'theta')