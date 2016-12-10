% This function senses arm state (in joint space) via a biased and noisy
% proprioceptive system, accounting for time delay via state augmentation.
% Note that this sensing is done in joint space regardless of the space
% set for control, which is only used for the purposes of MPC optimization.
function xSens = sense(arm)

% extract most delayed state from augmented state vector
nStates = length(arm.x.val);
xSens = arm.z.val(end-(nStates-1):end);

% add bias & noise
sensBias = computeBias(arm);
xSens = xSens + sensBias + arm.sensNoise;

end