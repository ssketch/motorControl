% This function senses the current arm state via a biased and noisy
% proprioceptive system. The state is extended to account for time delay in
% sensory feedback.
function y = sense(arm, x)

% extract most delayed state from augmented state vector 
if model
    y = params.Cext_mod*x;
else
    y = params.Cext_act*x;
end

% add bias & noise
    [bias, noise] = computeError(arm);
    noise = [params.noise.*randn(2,1);0;0];
    y = y + [bias;0;0] + noise;
end

end