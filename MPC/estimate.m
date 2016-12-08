function [xPnext, Pnext] = estimate(xP, P, y, T, params)
% This function uses the brain's internal model, along with an efference
% copy of the command sent to the arm and sensory feedback from the
% proprioceptive system, to update an estimate of the arm's state, as well
% as this estimate's uncertainty. The estimation is performed with an
% unscented Kalman filter, using the fully nonlinear dynamics with state
% augmented to account for time delay. 
%
% ___________                                         _________
% |         |                     u*                  |       |   x
% | Control |_________________________________________| Plant |____
% |_________|                     |                   |_______|
%       |                         |                       |
%       |       __________________|___________________    |
%       |       |                                    |    | y
%       |_______| Estimator                          |____|
%         x_est |   unscented Kalman filter (UKF):   |
%               |    1) predict arm state & sensory  |
%               |       feedback via internal model  |
%               |       & unscented transform (UT)   |
%               |       of sigma points              |
%               |    2) compute uncertainty in each  |
%               |       prediction                   |
%               |    3) update predicted state with  |
%               |       difference between actual &  |
%               |       predicted sensory feedback,  |
%               |       weighted according to        |
%               |       computed uncertainties       |
%               |____________________________________|

% nonlinear model + unscented Kalman filter
[xPnext, Pnext] = ukf(xP, P, y, T, @actuate, @sense, params);

end