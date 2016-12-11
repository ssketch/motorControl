% This function senses arm state (in joint space) via a biased and noisy
% proprioceptive system, accounting for time delay via state augmentation.
% Note that this sensing is done in joint space regardless of the space
% set for control, which is only used for the purposes of MPC optimization.
% Not that input 'z' is optional. If 'z' is omitted, sensing will be done
% using the current state of the arm object.
function x_sens = sense(arm, z)

% if no state is specified, use current arm state
if nargin < 2
    z = arm.z.val;
end

% extract most delayed state from augmented state vector
nStates = length(arm.x.val);
x_sens = z(end-(nStates-1):end);

% add bias & noise
sensBias = computeBias(arm);
x_sens = x_sens + sensBias + arm.sensNoise;

end