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
% The implementation pre-computes the screw axes (space representation) and
% the home configurations of every articulated body, then evaluates the
% forward kinematics with the Product-of-Exponentials formula.

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
        body = robot_struct.Bodies{idx};

        % Base-to-body transform at home configuration (M_i)
        M_i = getTransform(robot_struct, homeConfig, bodyName);
        Mlist{idx} = M_i;

        % Retrieve joint frame at home configuration
        childToJoint = body.Joint.ChildToJointTransform;
        T_base_joint = M_i * tforminv(childToJoint);

        % Joint axis expressed in joint frame
        axisLocal = body.Joint.JointAxis(:);
        axisNorm = norm(axisLocal);
        if axisNorm > eps
            axisLocal = axisLocal / axisNorm;
        end

        % Map axis (and point) to space frame
        R_base_joint = T_base_joint(1:3, 1:3);
        p_base_joint = T_base_joint(1:3, 4);

        switch lower(body.Joint.Type)
            case 'revolute'
                omega = R_base_joint * axisLocal;
                v = -cross(omega, p_base_joint);
                Slist(:, idx) = [omega; v];
            case 'prismatic'
                v = R_base_joint * axisLocal;
                Slist(:, idx) = [zeros(3, 1); v];
            otherwise
                % Fixed or unsupported joint types contribute no motion
                Slist(:, idx) = zeros(6, 1);
        end
    end
end

function g = expTwist(S, theta)
    omega = S(1:3);
    v = S(4:6);
    omegaNorm = norm(omega);

    if omegaNorm < 1e-9 % prismatic or pure translation
        R = eye(3);
        p = v * theta;
    else
        % Assume omega is already unit-length; re-normalize for safety
        omega = omega / omegaNorm;
        theta = theta * omegaNorm;
        omegaHat = skew(omega);
        R = eye(3) + sin(theta) * omegaHat + (1 - cos(theta)) * (omegaHat * omegaHat);
        G = eye(3) * theta + (1 - cos(theta)) * omegaHat + (theta - sin(theta)) * (omegaHat * omegaHat);
        p = G * (v / omegaNorm);
    end

    g = eye(4);
    g(1:3, 1:3) = R;
    g(1:3, 4) = p;
end

function S = skew(w)
    S = [  0     -w(3)  w(2);
          w(3)    0    -w(1);
         -w(2)  w(1)    0  ];
end
