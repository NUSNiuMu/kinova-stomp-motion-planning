% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)

nJoints = length(em);
[nSamples, nDiscretize] = size(trajProb);
dtheta = zeros(nJoints, nDiscretize);

% Unbiased weighting (baseline), per PPT probability slide
W = trajProb - 1/nSamples;

% Probability-weighted sum of sampled noises, per time-step
for j = 1:nJoints
    dtheta(j, :) = sum(W .* em{j}, 1);
end

end
