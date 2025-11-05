% Input:
%   nSamplePaths: the number of sample paths
%   sigma: sample covariance matrix (Rinv, size = (n-2)x(n-2))
%   theta: mean trajectory from last iteration (nJoints x nDiscretize)
% Output:
%   theta_paths: sampled trajectories
%   em: sampled Gaussian trajectory for each joint
function [theta_paths, em] = stompSamples(nSamplePaths, sigma, theta)

[nJoints, nDiscretize] = size(theta);

em = cell(1, nJoints);
ek = cell(1, nSamplePaths);
theta_paths = cell(1, nSamplePaths);

% Inner knots only (PPT finite-difference formulation keeps endpoints)
innerN = nDiscretize - 2;
mu_inner = zeros(1, innerN);

% Independent sampling for each joint, per PPT
for m = 1:nJoints
    % Sample inner points with covariance 'sigma' (Rinv)
    z_inner = mvnrnd(mu_inner, sigma, nSamplePaths);         % (nSamplePaths x innerN)
    % Embed into full length by padding zeros at endpoints (no noise at endpoints)
    z_full = zeros(nSamplePaths, nDiscretize);
    z_full(:, 2:nDiscretize-1) = z_inner;
    em{m} = z_full;                                          % (nSamplePaths x nDiscretize)
end

% Regroup by samples and add to mean trajectory
emk = [em{:}];                                               % (nSamplePaths x (nJoints*nDiscretize))
for k = 1:nSamplePaths
    dk = reshape(emk(k, :), nDiscretize, nJoints)';          % -> (nJoints x nDiscretize)
    ek{k} = dk;
    theta_paths{k} = theta + dk;
end

end
