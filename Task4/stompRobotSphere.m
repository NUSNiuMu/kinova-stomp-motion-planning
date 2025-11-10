%% Construct the spheres around the robot manipulator (Task4 local copy)
% INPUT:
%       X: joints (x,y,z,1) position in the world frame (Size: nJoints by 4)
% OUTPUT:
%       sphere centers and radi
function [centers, radi] = stompRobotSphere(X)
nJoints = size(X,1);
center_cell = cell(nJoints,1);
radi_cell = cell(nJoints,1);
% construct the spheres for all links
for k=1:size(X,1)
    if k==1
        parent_joint_position = [0,0,0]; 
    else
        parent_joint_position = X(k-1, 1:3);
    end
    child_joint_poisition = X(k, 1:3);
    % radius of the sphere (override from base workspace if provided)
    rad = 0.05;
    try
        if evalin('base','exist(''stomp_sphere_radius'',''var'')')
            rad = evalin('base','stomp_sphere_radius');
        end
    end
    % number of spheres
    nSpheres = ceil(norm(child_joint_poisition-parent_joint_position)/rad)+1;
    % centers of the spheres along the link
    center_cell_k = arrayfun(@(x1, x2) linspace(x1,x2,nSpheres), parent_joint_position', child_joint_poisition','UniformOutput', false);
    center_cell{k} = cell2mat(center_cell_k)';
    radi_cell{k} = rad*ones(size(center_cell{k},1),1);
end    

centers = cell2mat(center_cell);
radi = cell2mat(radi_cell);

end