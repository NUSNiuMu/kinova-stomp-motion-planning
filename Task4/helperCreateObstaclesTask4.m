%% Task4: Construct voxel obstacle representation with multiple obstacles
% Define the workspace (voxel world limits)
Env_size = [-1, -1, -1; 2, 2, 2];  % [xmin, ymin, zmin] and [xlen, ylen, zlen]
voxel_size = [0.02, 0.02, 0.02];    % meters
% Allow finer grid override from run script
if evalin('base','exist(''voxel_size_override'',''var'')')
    voxel_size = evalin('base','voxel_size_override');
end

% Binary map: all free initially (dimensions in order: X-by-Y-by-Z)
% Ensure integer grid sizes from metric lengths and voxel resolution
grid_size = max([1 1 1], floor(Env_size(2,:)./voxel_size));
binary_world = zeros(grid_size(1), grid_size(2), grid_size(3));

%% Define static obstacles (axis-aligned boxes)
% Default parameters
wall_len_x = 0.30; wall_thick_y = 0.06; wall_height_z = 0.25;
base_z = 0.30; gap_y_center_default = 0.38;
% Allow user overrides
gap_y = 0.10; % default gap width (meters)
if exist('corridor_gap_y','var'); gap_y = corridor_gap_y; end
if exist('corridor_wall_thick_y','var'); wall_thick_y = corridor_wall_thick_y; end

world = {};
boxes = {};
oriented_boxes = {};

% 仅使用 scenario_id 构建障碍（移除 corridor/zigzag/手动覆写）
if ~exist('scenario_id','var')
    error('Task4/helperCreateObstaclesTask4: 请在运行脚本中设置 scenario_id = 1/2/3/4');
end

switch scenario_id
    case 1
        % 场景1：斜墙阻断（两面倾斜墙形成角度走廊）
        alpha = deg2rad(25);
        Rz = axang2rotm([0 0 1 alpha]);
        Rz2 = axang2rotm([0 0 1 -20*pi/180]);
        dims = [0.35, 0.04, 0.25];
        c1 = [0.52, 0.1, 0.60];
        c2 = [0.62, 0.44, 0.30];
        w1 = collisionBox(dims(1), dims(2), dims(3)); w1.Pose = trvec2tform(c1) * rotm2tform(Rz);
        w2 = collisionBox(dims(1), dims(2), dims(3)); w2.Pose = trvec2tform(c2) * rotm2tform(Rz2);
        world{end+1} = w1; world{end+1} = w2;
        oriented_boxes(end+1,:) = {c1, dims, Rz};
        oriented_boxes(end+1,:) = {c2, dims, Rz2};
        boxes(end+1,:) = {[0.56, 0.40, 0.30], [0.12, 0.04, 0.20]};

    case 2
        % 场景2：高低障碍（低斜板 + 高横梁，需改变高度）
        beta = deg2rad(25);
        Ry = axang2rotm([0 1 0 beta]);
        dims1 = [0.40, 0.20, 0.06];
        c1 = [0.55, 0.05, 0.62];
        slab = collisionBox(dims1(1), dims1(2), dims1(3)); slab.Pose = trvec2tform(c1) * rotm2tform(Ry);
        world{end+1} = slab; oriented_boxes(end+1,:) = {c1, dims1, Ry};
        boxes(end+1,:) = {[0.60, 0.42, 0.48], [0.35, 0.25, 0.06]};

    case 3
        % 场景3：组合场景（case1 + case2）
        alpha = deg2rad(25);
        Rz = axang2rotm([0 0 1 alpha]);
        Rz2 = axang2rotm([0 0 1 -20*pi/180]);
        dims_z = [0.35, 0.04, 0.25];
        c1_z = [0.52, 0., 0.60];
        c2_z = [0.62, 0.44, 0.30];
        w1 = collisionBox(dims_z(1), dims_z(2), dims_z(3)); w1.Pose = trvec2tform(c1_z) * rotm2tform(Rz);
        w2 = collisionBox(dims_z(1), dims_z(2), dims_z(3)); w2.Pose = trvec2tform(c2_z) * rotm2tform(Rz2);
        world{end+1} = w1; world{end+1} = w2;
        oriented_boxes(end+1,:) = {c1_z, dims_z, Rz};
        oriented_boxes(end+1,:) = {c2_z, dims_z, Rz2};
        boxes(end+1,:) = {[0.56, 0.40, 0.30], [0.12, 0.04, 0.20]};

        beta = deg2rad(25);
        Ry = axang2rotm([0 1 0 beta]);
        dims_y = [0.40, 0.20, 0.06];
        c1_y = [0.35, 0.3, 0.7];
        slab = collisionBox(dims_y(1), dims_y(2), dims_y(3)); slab.Pose = trvec2tform(c1_y) * rotm2tform(Ry);
        world{end+1} = slab; oriented_boxes(end+1,:) = {c1_y, dims_y, Ry};
        boxes(end+1,:) = {[0.60, 0.42, 0.48], [0.35, 0.25, 0.06]};

    case 4
        % 场景4：弧形拱桥（沿 Y–Z 平面布置拱砖 + 两端立柱）
        arch_center = [0.60, 0.38, 0.40];
        arch_radius = 0.16;
        arch_theta_span = deg2rad([0, 180]);
        N = 9;
        dimsB = [0.20, 0.04, 0.06];
        xs = arch_center(1);
        arch_center(3) = arch_center(3) + 0.30;
        thetas = linspace(arch_theta_span(1), arch_theta_span(2), N);
        for i = 1:N
            th = thetas(i);
            y = arch_center(2) + arch_radius * cos(th);
            z = arch_center(3) + arch_radius * sin(th);
            c = [xs, y, z];
            phi = th + pi/2;
            Rx = axang2rotm([1 0 0 phi]);
            cb = collisionBox(dimsB(1), dimsB(2), dimsB(3));
            cb.Pose = trvec2tform(c) * rotm2tform(Rx);
            world{end+1} = cb; oriented_boxes(end+1,:) = {c, dimsB, Rx};
        end
        pillar_dims = [0.10, 0.10, 0.22];
        left_base  = [xs, arch_center(2) - arch_radius, arch_center(3) - pillar_dims(3)/2];
        right_base = [xs, arch_center(2) + arch_radius, arch_center(3) - pillar_dims(3)/2];
        pl = collisionBox(pillar_dims(1), pillar_dims(2), pillar_dims(3)); pl.Pose = trvec2tform(left_base);
        pr = collisionBox(pillar_dims(1), pillar_dims(2), pillar_dims(3)); pr.Pose = trvec2tform(right_base);
        world{end+1} = pl; boxes(end+1,:) = {left_base,  pillar_dims};
        world{end+1} = pr; boxes(end+1,:) = {right_base, pillar_dims};

    otherwise
        error('Task4/helperCreateObstaclesTask4: 无效的 scenario_id，使用 1/2/3/4');
end

%% Voxelize obstacles into binary_world
xmin = Env_size(1,1); ymin = Env_size(1,2); zmin = Env_size(1,3);

% Boxes to voxelize are collected in 'boxes' and 'oriented_boxes'

for b = 1:size(boxes,1)
    c = boxes{b,1}; d = boxes{b,2};
    xmin_b = c(1) - d(1)/2; xmax_b = c(1) + d(1)/2;
    ymin_b = c(2) - d(2)/2; ymax_b = c(2) + d(2)/2;
    zmin_b = c(3) - d(3)/2; zmax_b = c(3) + d(3)/2;

    xi1 = max(1, ceil((xmin_b - xmin)/voxel_size(1))); xi2 = min(size(binary_world,1), floor((xmax_b - xmin)/voxel_size(1)));
    yi1 = max(1, ceil((ymin_b - ymin)/voxel_size(2))); yi2 = min(size(binary_world,2), floor((ymax_b - ymin)/voxel_size(2)));
    zi1 = max(1, ceil((zmin_b - zmin)/voxel_size(3))); zi2 = min(size(binary_world,3), floor((zmax_b - zmin)/voxel_size(3)));

    if xi1<=xi2 && yi1<=yi2 && zi1<=zi2
        binary_world(xi1:xi2, yi1:yi2, zi1:zi2) = 1;
    end
end

% Voxelize oriented boxes
for b = 1:size(oriented_boxes,1)
    c = oriented_boxes{b,1}; d = oriented_boxes{b,2}; R = oriented_boxes{b,3};
    binary_world = helperVoxelizeOrientedBox(binary_world, Env_size, voxel_size, c, d, R);
end

%% Signed Euclidean Distance Transform (meters)
voxel_world_sEDT = sEDT_3d(binary_world) .* voxel_size(1);

% Pack voxel world struct
voxel_world.world_size = size(binary_world);
voxel_world.Env_size   = Env_size;
voxel_world.voxel_size = voxel_size; % keep full [x y z] vector
voxel_world.sEDT       = voxel_world_sEDT;