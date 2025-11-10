function [theta_paths, em]=stompSamples(nSamplePaths,sigma,theta)
% Independent sampling for each joint (Task4 local copy)

[nJoints, nDiscretize] = size(theta);

em = cell(1,nJoints);
ek = cell(1,nSamplePaths);

theta_paths = cell(1, nSamplePaths);
mu=zeros(1,size(sigma, 1)); % size of sigma is (nDiscretize-2) x (nDiscretize-2)

%% Independent sampling
for m = 1 : nJoints
    A = chol(sigma, 'lower'); 
    % 2. Sample random normal vector: Z is ((nDiscretize-2) x nSamplePaths)
    Z = randn(nDiscretize-2, nSamplePaths);
    % 3. Sample: E = mu + A * Z
    % em_m is (nSamplePaths x (nDiscretize-2))
    em_m = (A * Z)' + mu;
    
    % Store the noise (em_m) for the current joint m
    % em is a cell array (1 x numJoints), each cell is (nPaths x (nDiscretize-2))
    em{m} = em_m;
end

%% Regroup by samples
emk = [em{:}]; % Flattens all joint samples into one matrix (nPaths x numJoints*(nDiscretize-2))
for k=1:nSamplePaths
    % Reshape the noise for path k: (numJoints x (nDiscretize-2))
    ek_movable = reshape(emk(k,:), nDiscretize-2, nJoints)'; 
    
    % Pad with zeros for fixed q0 and qT: (numJoints x nDiscretize)
    ek_k = zeros(nJoints, nDiscretize);
    ek_k(:, 2:nDiscretize-1) = ek_movable;
    
    theta_paths{k} = theta + ek_k;
end