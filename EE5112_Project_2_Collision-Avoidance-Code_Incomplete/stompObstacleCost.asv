function cost = stompObstacleCost(sphere_centers, radius, voxel_world, vel)

safety_margin = 0.1; % safety margin (m)
cost = 0;

% signed distance field (SDF) per PPT (EDT and signed-EDT)
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;

% Map sphere centers to voxel indices
env_corner = voxel_world.Env_size(1,:);                                 % [xmin ymin zmin]
env_corner_vec = repmat(env_corner, length(sphere_centers), 1);
idx = ceil((sphere_centers - env_corner_vec) ./ voxel_world.voxel_size);

try
    % clamp indices into valid volume
    lin_idx = sub2ind(world_size, ...
                      max(1, min(world_size(1), idx(:,1))), ...
                      max(1, min(world_size(2), idx(:,2))), ...
                      max(1, min(world_size(3), idx(:,3))));

    % distance to nearest obstacle boundary minus sphere radius
    d = voxel_world_sEDT(lin_idx) - radius;

    % per PPT Eq(“collision potential”): positive inside safety band, zero outside
    phi = max(0, safety_margin - d);

    % velocity weight per time-step (|dq/dt| projected), as in PPT notes
    speed = vecnorm(vel, 2, 2);

    % total obstacle cost (per time-step, then summed)
    cost_array = (phi.^2) .* speed;     % quadratic penalty inside safety band
    cost = sum(cost_array);

catch
    % debug fall-back: recompute idx if any invalid index happens
    idx = ceil((sphere_centers - env_corner_vec) ./ voxel_world.voxel_size);
end
