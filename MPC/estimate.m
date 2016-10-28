% This function uses the brain's internal model, along with an efference
% copy of the torque sent to the arm and sensory feedback from the
% proprioceptive system, to update an estimate of the arm's state, as well
% as this estimate's uncertainty. The estimation is performed with a
% standard unscented Kalman filter, using the fully nonlinear arm dynamics.
function [xPnext, Pnext] = estimate(xP, P, y, T, params)
    
% nonlinear model + unscented Kalman filter
[xPnext, Pnext] = ukf(xP, P, y, T, @actuate, @sense, params);

end