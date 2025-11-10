function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)
% Safety margin (meters) with base-workspace override
safety_margin = 0.05;
try
    if evalin('base','exist(''stomp_safety_margin'',''var'')')
        safety_margin = evalin('base','stomp_safety_margin');
    end
end
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
%% Obstacle penalty per STOMP Eq (13)
try
    % bound indices
    idx(:,1) = max(1, min(idx(:,1), world_size(1)));
    idx(:,2) = max(1, min(idx(:,2), world_size(2)));
    idx(:,3) = max(1, min(idx(:,3), world_size(3)));

    linear_indices = sub2ind(world_size, idx(:,1), idx(:,2), idx(:,3));
    s_i = voxel_world_sEDT(linear_indices);

    d_i = s_i - radius; % effective clearance minus sphere radius
    delta_i = safety_margin - d_i;
    valid_indices = delta_i > 0;

    % Penalty strength (alpha) with base-workspace override
    alpha = 200;
    try
        if evalin('base','exist(''stomp_alpha'',''var'')')
            alpha = evalin('base','stomp_alpha');
        end
    end
    cost_array = zeros(size(sphere_centers, 1), 1);
    delta_i_valid = delta_i(valid_indices);
    cost_array(valid_indices) = exp(alpha * delta_i_valid.^2) - 1;
    cost = sum(cost_array);
catch
    cost = 0;
end