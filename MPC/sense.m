% This function senses the current arm state via a biased and noisy
% proprioceptive system. The state is extended to account for time delay in
% sensory feedback.
function y = sense(x, params)

% determine whether working with real arm or arm model
model = floor(length(x)/params.Nstates) == params.numDelSteps_mod;

% sense output (i.e., most delayed state) via proprioception
if model
    y = params.Cext_mod*x;
else
    y = params.Cext_act*x;
end

% add bias & noise for simulation
if ~model
    th = x(1:2);
    bias = computeBias(th, params.biasSlope, params.biasInter);
    noise = [params.noise.*randn(2,1);0;0];
    y = y + [bias;0;0] + noise;
end

end

if ~model
    xNext(1:params.Nstates) = xNext(1:params.Nstates) + ...
        sqrt(diag(params.Q)).*randn(params.Nstates,1);
end