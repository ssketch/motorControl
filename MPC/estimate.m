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
%       |       |                                    |    | x_sens
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
%
function x_est = estimate(intModel, u, x_sens)

% perform unscented Kalman filtering
[z_est, P] = ukf(intModel, x_sens, u, @plant, @sense);

% extract estimate of current state from augmented state estimate
nStates = length(intModel.x.val);
x_est = z_est(1:nStates);

% update state variables for internal model object
nJoints = length(intModel.q.val);
intModel.u.val = u;
intModel.x.val = x_est;
intModel.q.val = x_est(1:nJoints)';
[intModel.y.val, intModel.elbow, ~] = fwdKin(intModel);
intModel.z.val = z_est;
intModel.P = P;

end