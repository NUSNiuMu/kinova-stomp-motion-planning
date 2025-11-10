%% forward kinematics (Task4 local copy)
% INPUT:
%       robot_struct: the Matlab robot structure object
%       theta: the joints rotation angles
% OUTPUT:
%       X: Joints' positions in the world frame
%       T: Homogeneous Transformation from the Joint frame to the base frame
function [X, T] = updateJointsWorldPosition(robot_struct, theta)

% Update the robot configuration structure used by Matlab
theta_cell = num2cell(theta);
tConfiguration = robot_struct.homeConfiguration;   % copy struct
[tConfiguration.JointPosition] = theta_cell{:};    % update joint positions

% get the number of joints
nJoints = length(theta);
T = cell(1, nJoints);
X = zeros(nJoints, 4);

% Per time step: compute transform and joint position
for k = 1:nJoints
    T{k} = getTransform(robot_struct, tConfiguration, robot_struct.BodyNames{k});
    p = T{k}(1:3, 4).';
    X(k, :) = [p 1];
end

end