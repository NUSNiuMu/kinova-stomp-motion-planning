% dtheta: estimated gradient
% em: 1 by nJoints cell, each cell is nSamples by nDiscretize matrix
function dtheta = stompDTheta(trajProb, em)



nJoints = length(em);
[~, nDiscretize] = size(trajProb);
dtheta = zeros(nJoints, nDiscretize);



% Probability-weighted sum of sampled noises, per time-step
for j = 1:nJoints
    dtheta(j,:) = sum(trajProb .* em{j}, 1);
end

end
