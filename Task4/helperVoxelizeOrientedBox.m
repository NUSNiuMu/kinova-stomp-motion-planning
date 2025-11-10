function BW = helperVoxelizeOrientedBox(BW, Env_size, voxel_size, center, dims, R)
% Voxelize a rotated (oriented) box into binary grid BW.
% BW: X-by-Y-by-Z logical/binary array
% Env_size: [xmin ymin zmin; xlen ylen zlen]
% voxel_size: [vx vy vz]
% center: [cx cy cz]
% dims: [dx dy dz]
% R: 3x3 rotation matrix of the box (world orientation)

xmin = Env_size(1,1); ymin = Env_size(1,2); zmin = Env_size(1,3);
vx = voxel_size(1); vy = voxel_size(2); vz = voxel_size(3);

% Compute oriented box corners in world
half = dims(:)'/2;
localCorners = [
    -half(1) -half(2) -half(3);
    -half(1) -half(2)  half(3);
    -half(1)  half(2) -half(3);
    -half(1)  half(2)  half(3);
     half(1) -half(2) -half(3);
     half(1) -half(2)  half(3);
     half(1)  half(2) -half(3);
     half(1)  half(2)  half(3)];
worldCorners = (R * localCorners')' + center;

% Axis-aligned bounding box indices
minC = min(worldCorners, [], 1);
maxC = max(worldCorners, [], 1);
xi1 = max(1, ceil((minC(1) - xmin) / vx));
xi2 = min(size(BW,1), floor((maxC(1) - xmin) / vx));
yi1 = max(1, ceil((minC(2) - ymin) / vy));
yi2 = min(size(BW,2), floor((maxC(2) - ymin) / vy));
zi1 = max(1, ceil((minC(3) - zmin) / vz));
zi2 = min(size(BW,3), floor((maxC(3) - zmin) / vz));

if xi1>xi2 || yi1>yi2 || zi1>zi2
    return;
end

% Iterate voxels in AABB and test membership via inverse transform
Rinv = R';
halfVec = half;
for xi = xi1:xi2
    wx = xmin + (xi - 0.5) * vx;
    for yi = yi1:yi2
        wy = ymin + (yi - 0.5) * vy;
        for zi = zi1:zi2
            wz = zmin + (zi - 0.5) * vz;
            p = [wx wy wz] - center;
            q = Rinv * p';
            q = q';
            if all(abs(q) <= halfVec + 1e-8)
                BW(xi, yi, zi) = 1;
            end
        end
    end
end