function [centers, radii] = stompRobotSpheresFullArm(X_in, n_per_link, link_radius)
% 兼容 X_in 格式:
%  - 3xN 位置矩阵（每列一个关节/末端点）
%  - N x 3 位置矩阵
%  - 4x4xN 齐次变换序列（取列4的平移）
%  - 1xN / Nx1 单元格，每个是 3x1 向量或 4x4 齐次变换
%
% 输出:
%  centers: (nLinks*n_per_link) x 3
%  radii:   (nLinks*n_per_link) x 1

% ---- 统一为 3xN ----
X = asPositions3xN(X_in);          % <== 关键：转换位置为 3xN
nLinks = size(X,2) - 1;
if nLinks < 1
    centers = zeros(0,3); radii = zeros(0,1); return;
end

% ---- 参数默认值 ----
if nargin < 2 || isempty(n_per_link),  n_per_link  = 5;   end
if nargin < 3 || isempty(link_radius), link_radius = 0.04; end  % 标量或 1×nLinks

if isscalar(link_radius)
    link_radius = repmat(link_radius, 1, nLinks);
elseif numel(link_radius) ~= nLinks
    error('link_radius 需为标量或长度为 nLinks 的向量。');
end

% ---- 逐连杆放置等间距小球（避开端点以免重复）----
N = nLinks * n_per_link;
centers = zeros(N,3);
radii   = zeros(N,1);
k = 1;
for ell = 1:nLinks
    p0 = X(:,ell);
    p1 = X(:,ell+1);
    for s = 1:n_per_link
        t = (s - 0.5) / n_per_link;       % 段内中心
        p  = (1 - t).*p0 + t.*p1;
        centers(k,:) = p.';
        radii(k,1)   = link_radius(ell);
        k = k + 1;
    end
end
end

% --------- 工具函数：把各种 X 格式转成 3xN 位置矩阵 ----------
function P = asPositions3xN(X)
    % 3xN
    if isnumeric(X) && ismatrix(X) && size(X,2)==4
        P = X(:,1:3).';     % 转为 3×N
        return;
    end
    if isnumeric(X) && ismatrix(X) && size(X,1)==3
        P = X; return;
    end
    % N x 3
    if isnumeric(X) && ismatrix(X) && size(X,2)==3
        P = X.'; return;
    end
    % 4x4xN 齐次变换
    if isnumeric(X) && ndims(X)==3 && size(X,1)==4 && size(X,2)==4
        N = size(X,3);
        P = zeros(3,N);
        for i=1:N, P(:,i) = X(1:3,4,i); end
        return;
    end
    % 单元格：每个是 3x1 或 4x4
    if iscell(X)
        N = numel(X);
        P = zeros(3,N);
        for i=1:N
            Xi = X{i};
            if isnumeric(Xi) && all(size(Xi)==[3,1])
                P(:,i) = Xi;
            elseif isnumeric(Xi) && all(size(Xi)==[4,4])
                P(:,i) = Xi(1:3,4);
            else
                error('单元格第 %d 项既不是 3x1 也不是 4x4。', i);
            end
        end
        return;
    end
    error('无法识别 X 的格式，请提供 3xN / N×3 / 4x4xN / cell{ }。');
end
