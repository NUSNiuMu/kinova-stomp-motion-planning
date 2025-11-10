% dtheta: estimated raw gradient
% trajProb: nPaths x (nDiscretize-2) matrix of probabilities P_k for each waypoint
% em: 1 by nJoints cell, each cell is nSamples by (nDiscretize-2) matrix of noise epsilon
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
nDiscretize_movable = size(trajProb, 2); % nDiscretize-2
nPaths = size(trajProb, 1);
% variable declaration
dtheta = zeros(nJoints, nDiscretize_movable);

%% TODO: iterate over all joints to compute dtheta: (complete your code according to the STOMP algorithm)
for m = 1:nJoints
    % em{m} is the noise for joint m: (nPaths x nDiscretize_movable)
    em_m = em{m};
    
    % Hadamard product (element-wise multiplication) between P^k and epsilon^k
    % Weighted_noise: (nPaths x nDiscretize_movable)
    weighted_noise = trajProb .* em_m;
    
    % Sum over all sampled paths (k) to get the raw gradient estimate (Formula 17)
    % dtheta_m_raw: (1 x nDiscretize_movable)
    dtheta_m_raw = sum(weighted_noise, 1);
    
    % Store the result for joint m
    dtheta(m, :) = dtheta_m_raw;
end