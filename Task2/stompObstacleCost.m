function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)
safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
%% TODO: complete the following code according to Eq (13) in the STOMP conference paper.
try
    idx(:,1) = max(1, min(idx(:,1), world_size(1)));
    idx(:,2) = max(1, min(idx(:,2), world_size(2)));
    idx(:,3) = max(1, min(idx(:,3), world_size(3)));
    linear_indices = sub2ind(world_size, idx(:,1), idx(:,2), idx(:,3));
    s_i = voxel_world_sEDT(linear_indices);
    d_i = s_i - radius;
    delta_i = safety_margin - d_i;
    valid_indices = delta_i > 0;
    alpha = 100; 
    cost_array = zeros(size(sphere_centers, 1), 1);
    delta_i_valid = delta_i(valid_indices);
    cost_array(valid_indices) = exp(alpha * delta_i_valid.^2) - 1;


    cost_exp = sum(cost_array);



    lin_idx = sub2ind(world_size, ...
                      max(1, min(world_size(1), idx(:,1))), ...
                      max(1, min(world_size(2), idx(:,2))), ...
                      max(1, min(world_size(3), idx(:,3))));

    d = voxel_world_sEDT(lin_idx) - radius;

    % per PPT Eq(“collision potential”): positive inside safety band, zero outside
    phi = max(0, safety_margin - d);

    % velocity weight per time-step (|dq/dt| projected), as in PPT notes
    speed = vecnorm(vel, 2, 2);

    % total obstacle cost (per time-step, then summed)
    cost_array2 = (phi.^2) .* speed;     % quadratic penalty inside safety band

    cost_liner = sum(cost_array2)*10000;

    cost = cost_liner;


catch % for debugging
    % 保持原有的调试代码逻辑
    idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
    % 遇到错误时，返回 0 成本或进行其他错误处理
    cost_array = zeros(size(sphere_centers, 1), 1); 
    cost = 0;
end