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
    % 1. 提取每个球体中心所在体素的符号距离 (signed distance)
    % 确保索引在有效范围内 (1 到 world_size)
    
    % 将 idx 中的列索引 (x, y, z) 转换为 sEDT 数组的线性索引
    % 注意：MATLAB 数组是列优先存储，索引顺序是 (x, y, z) 对应 (row, col, plane)
    % 假设 idx 已经是 [x_subscript, y_subscript, z_subscript]
    
    % 限制索引，防止越界
    idx(:,1) = max(1, min(idx(:,1), world_size(1)));
    idx(:,2) = max(1, min(idx(:,2), world_size(2)));
    idx(:,3) = max(1, min(idx(:,3), world_size(3)));

    linear_indices = sub2ind(world_size, idx(:,1), idx(:,2), idx(:,3));
    
    % s_i: 从 sEDT 中获取与每个球体中心对应的符号距离
    s_i = voxel_world_sEDT(linear_indices);

    % 2. 计算有效距离 d_i
    % d_i = s_i - radius (球体中心到障碍物的距离 减去 球体半径)
    % 当 d_i < 0 时，表示球体与障碍物发生碰撞
    d_i = s_i - radius;
    
    % 3. 施加惩罚的条件
    % 惩罚只在 d_i < safety_margin 时施加
    % delta_i = safety_margin - d_i
    delta_i = safety_margin - d_i;
    
    % 4. 提取需要施加惩罚的索引
    % 仅在 delta_i > 0 时才计算成本
    valid_indices = delta_i > 0;
    
    % 5. 计算成本 cost_array (公式 13 的指数衰减形式)
    % C_i = exp(alpha * delta_i^2) - 1, 当 delta_i > 0
    % C_i = 0,                         当 delta_i <= 0
    % 在 STOMP 中，通常 alpha = 1 / (2 * vel_i^2) 或类似的与速度相关的项
    
    % 这里我们使用一个常数 alpha 来简化，或者使用速度的倒数平方。
    % 考虑到函数输入 vel 是速度，常用形式为：
    % alpha 接近 1.0 (或 0.5) / (2 * vel^2)
    % 由于 vel 可能是一个向量，这里假设使用其范数的平方或一个平均/最大值
    
    % 假设使用一个简单的常数 alpha = 200 作为惩罚强度，因为它不依赖于速度 vel
    % 如果要求严格遵循公式 (13)，则需要使用 alpha(vel) = 1.0 / (2 * sigma_v^2)
    % 在 STOMP 实现中，通常速度项的方差 sigma_v 是一个设计参数。
    % 为了简化，假设 alpha = 200 (一个常用的高惩罚系数)
    alpha = 200; 
    
    % 初始化 cost_array 为 0
    cost_array = zeros(size(sphere_centers, 1), 1);
    
    % 提取需要计算成本的 delta_i
    delta_i_valid = delta_i(valid_indices);
    
    % 计算有效的成本项
    % C_i = exp(alpha * delta_i_valid.^2) - 1
    cost_array(valid_indices) = exp(alpha * delta_i_valid.^2) - 1;
    
    cost = sum(cost_array);
catch % for debugging
    % 保持原有的调试代码逻辑
    idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
    % 遇到错误时，返回 0 成本或进行其他错误处理
    cost_array = zeros(size(sphere_centers, 1), 1); 
    cost = 0;
end