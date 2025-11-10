% Task5: Trajectory cost with end-effector orientation constraint
function [Stheta, Qtheta] = stompTrajCost(robot_struct, theta,  R, voxel_world)
% Compute local trajectory cost Stheta (per discretization point)
% and overall trajectory cost Qtheta, including obstacle cost and EE orientation constraint.

% Discretization size
[~, nDiscretize] = size(theta);

% Costs
qo_cost = zeros(1, nDiscretize); % obstacle cost per step
qc_cost = zeros(1, nDiscretize); % constraint cost per step

% Obstacle cost (reuse Task4 logic)
[X, ~] = updateJointsWorldPosition(robot_struct, theta(:, 1));
[sphere_centers, radi] = stompRobotSphere(X);
vel = zeros(length(sphere_centers), 1);
qo_cost(1) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);

% Orientation constraint parameters (from base workspace)
task5_enable = false; keep_axis = 'y'; world_axis = [0;0;1]; penalty_w = 300; penalty_norm = 'l1';
penalty_metric = 'angle'; % 'angle' (default) or 'vec_l1'/'residual_l1' or 'angle_hinge'
angle_threshold_deg = 10; % for 'angle_hinge' metric
try
    if evalin('base','exist(''task5_enable'',''var'')'), task5_enable = evalin('base','task5_enable'); end
    if evalin('base','exist(''task5_keep_axis'',''var'')'), keep_axis = evalin('base','task5_keep_axis'); end
    if evalin('base','exist(''task5_world_axis'',''var'')'), world_axis = evalin('base','task5_world_axis'); end
    if evalin('base','exist(''task5_penalty_weight'',''var'')'), penalty_w = evalin('base','task5_penalty_weight'); end
    if evalin('base','exist(''task5_penalty_norm'',''var'')'), penalty_norm = evalin('base','task5_penalty_norm'); end
    if evalin('base','exist(''task5_penalty_metric'',''var'')'), penalty_metric = evalin('base','task5_penalty_metric'); end
    if evalin('base','exist(''task5_angle_threshold_deg'',''var'')'), angle_threshold_deg = evalin('base','task5_angle_threshold_deg'); end
end
% Normalize world axis
if numel(world_axis) == 3 && norm(world_axis) > eps
    world_axis = world_axis(:) / norm(world_axis);
else
    world_axis = [0;0;1];
end
% Axis index in EE frame
switch lower(string(keep_axis))
    case "x", ax_idx = 1;
    case "y", ax_idx = 2;
    case "z", ax_idx = 3;
    otherwise, ax_idx = 2; % default y-axis
end
eeName = robot_struct.BodyNames{end};

for i = 2 : nDiscretize
    % Previous sphere centers for velocity approx
    sphere_centers_prev = sphere_centers;
    % Kinematics for obstacle cost
    [X, ~] = updateJointsWorldPosition(robot_struct, theta(:, i));
    [sphere_centers, radi] = stompRobotSphere(X);
    vel = vecnorm(sphere_centers_prev - sphere_centers, 2, 2);
    qo_cost(i) = stompObstacleCost(sphere_centers, radi, voxel_world, vel);

    % Orientation constraint cost
    if task5_enable
        % Build configuration struct
        theta_cell = num2cell(theta(:, i));
        tConfiguration = robot_struct.homeConfiguration;
        [tConfiguration.JointPosition] = theta_cell{:};
        % EE transform and rotation
        Tee = getTransform(robot_struct, tConfiguration, eeName);
        Ree = Tee(1:3, 1:3);
        ee_axis_world = Ree(:, ax_idx);
        if norm(ee_axis_world) > eps
            ee_axis_world = ee_axis_world / norm(ee_axis_world);
        end
        % Orientation constraint cost metric
        switch lower(string(penalty_metric))
            case {"vec_l1","residual_l1"}
                % Vector residual L1: ||ee_axis_world - world_axis||_1
                residual = ee_axis_world - world_axis;
                qc_cost(i) = penalty_w * norm(residual, 1);
            case "angle_hinge"
                % Angle hinge: penalize only angle beyond a threshold
                dotv = max(-1, min(1, dot(ee_axis_world, world_axis)));
                ang = acos(dotv); % radians
                ang_thr = max(0, angle_threshold_deg) * pi/180; % radians
                hinge = max(0, ang - ang_thr);
                % Use squared hinge for stronger enforcement beyond threshold
                qc_cost(i) = penalty_w * (hinge.^2);
            otherwise
                % Angle-based penalty (legacy): use angle between axes
                dotv = max(-1, min(1, dot(ee_axis_world, world_axis)));
                ang = acos(dotv); % radians in [0, pi]
                switch lower(string(penalty_norm))
                    case "l2"
                        qc_cost(i) = penalty_w * (ang.^2);
                    otherwise % l1 or fallback
                        qc_cost(i) = penalty_w * abs(ang);
                end
        end
    else
        qc_cost(i) = 0;
    end
end

% Local trajectory cost: weights between obstacle/constraint
Stheta = 1000*qo_cost + qc_cost;

% Overall cost: sum over time plus smoothness term
theta_movable = theta(:, 2:end-1);
Qtheta = sum(Stheta) + 1/2 * sum(theta_movable * R * theta_movable', "all");

end