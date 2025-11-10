% dtheta: estimated raw gradient (Task4 local copy)
% trajProb: nPaths x (nDiscretize-2) matrix of probabilities P_k for each waypoint
% em: 1 by nJoints cell, each cell is nSamples by (nDiscretize-2) matrix of noise epsilon
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize_movable = size(trajProb, 2); % nDiscretize-2
nPaths = size(trajProb, 1);
% variable declaration
dtheta = zeros(nJoints, nDiscretize_movable);

% Iterate over all joints to compute dtheta per STOMP
for m = 1:nJoints
    % em{m} is the noise for joint m: (nPaths x nDiscretize_movable)
    em_m = em{m};
    % Weighted noise by path probability
    weighted_noise = trajProb .* em_m; % (nPaths x nDiscretize_movable)
    % Sum over sampled paths k to get raw gradient estimate
    dtheta_m_raw = sum(weighted_noise, 1);
    % Store
    dtheta(m, :) = dtheta_m_raw;
end

end