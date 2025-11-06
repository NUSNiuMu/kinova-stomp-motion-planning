%% forward kinematics
% INPUT:
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)
%UPDATEJOINTSWORLDPOSITION Forward kinematics using PoE (Task 3 requirement)
% INPUT:
%       robot_struct: rigidBodyTree object for the chosen manipulator
%       theta:        joint positions (n x 1 or 1 x n)
% OUTPUT:
%       X:            homogeneous joint positions in world frame (n x 4)
%       T:            base-to-body homogeneous transforms for each joint (1 x n cell)
%
% IMPLEMENTATION DETAILS:
% 1. Pre-computes screw axes S_i (space frame) from geometric Jacobian at home config
% 2. Caches home configurations M_i for each body (computed once via getTransform)
% 3. Applies Product-of-Exponentials: T_i = exp([S_1]θ_1)...exp([S_i]θ_i) * M_i
% 4. Uses persistent cache to avoid recomputation across multiple calls
%
% NOTE: Initial computation uses getTransform() once per joint to establish M_i,
%       consistent with textbook recommendations and efficiency requirements.

theta = theta(:); % ensure column vector
nJoints = length(theta);

% Cache PoE parameters per robot to avoid recomputation
persistent cachedSignature cachedS cachedM cachedNJoints
robotSignature = strjoin(robot_struct.BodyNames, '|');
if isempty(cachedSignature) || ~strcmp(cachedSignature, robotSignature) || ...
        isempty(cachedNJoints) || cachedNJoints ~= nJoints
    [cachedS, cachedM] = computePoEParameters(robot_struct, nJoints);
    cachedSignature = robotSignature;
    cachedNJoints = nJoints;
end

Slist = cachedS;
Mlist = cachedM;

% Accumulate the motion using PoE
T = cell(1, nJoints);
X = zeros(nJoints, 4);
g = eye(4);
for k = 1:nJoints
    g = g * expTwist(Slist(:, k), theta(k));
    T{k} = g * Mlist{k};
    p = T{k}(1:3, 4).';
    X(k, :) = [p 1];
end

end

%% Local helper functions ------------------------------------------------
function [Slist, Mlist] = computePoEParameters(robot_struct, nJoints)
    % Pre-compute screw axes (space frame) and home configurations M_i
    homeConfig = robot_struct.homeConfiguration;

    Slist = zeros(6, nJoints);
    Mlist = cell(1, nJoints);

    for idx = 1:nJoints
        bodyName = robot_struct.BodyNames{idx};

        % Base-to-body transform at home configuration (M_i)
        M_i = getTransform(robot_struct, homeConfig, bodyName);
        Mlist{idx} = M_i;

        % Space screw axis equals the idx-th column of the geometric Jacobian
        Jspace = geometricJacobian(robot_struct, homeConfig, bodyName);
        Slist(:, idx) = Jspace(:, idx);
    end
end

function g = expTwist(S, theta)
    % Compute matrix exponential of twist [S]*theta
    % Based on Rodrigues' formula for SE(3)
    % S = [ω; v] is 6x1 twist vector
    
    omega = S(1:3);
    v = S(4:6);
    omegaNorm = norm(omega);

    if omegaNorm < 1e-9 % prismatic joint (pure translation)
        R = eye(3);
        p = v * theta;
    else % revolute joint (rotation + translation)
        % Normalize angular velocity for numerical stability
        omega = omega / omegaNorm;
        theta = theta * omegaNorm;
        
        % Rodrigues' formula: R = I + sin(θ)[ω] + (1-cos(θ))[ω]²
        omegaHat = skew(omega);
        omegaHat2 = omegaHat * omegaHat;
        R = eye(3) + sin(theta) * omegaHat + (1 - cos(theta)) * omegaHat2;
        
        % G-matrix for translation: G = I*θ + (1-cos(θ))[ω] + (θ-sin(θ))[ω]²
        G = eye(3) * theta + (1 - cos(theta)) * omegaHat + (theta - sin(theta)) * omegaHat2;
        p = G * (v / omegaNorm);
    end

    % Construct homogeneous transformation matrix
    g = [R, p; 0, 0, 0, 1];
end

function S = skew(w)
    % Skew-symmetric matrix representation of 3D vector w
    % [w] = skew(w) such that [w]x = w × x (cross product)
    S = [  0     -w(3)  w(2);
          w(3)    0    -w(1);
         -w(2)  w(1)    0  ];
end
