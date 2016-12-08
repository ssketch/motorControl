% This function senses arm state via a biased and noisy proprioceptive
% system, accounting for time delay via state augmentation.
function y = sense(arm, x)

% extract most delayed state from augmented state vector 
% p.C = zeros(p.Nsensed,p.Nstates);
% p.C(:,1:p.Nsensed) = eye(p.Nsensed);
% 
% p.Cext_act = [zeros(p.Nsensed,p.Nstates*p.numDelSteps) p.C zeros(p.Nsensed,1)];     % actual time delay

% add bias & noise
sensBias = computeBias(arm);
y = y + sensBias + arm.sensNoise;

end