% This function uses a model of the arm, a copy of the command sent to the
% actual arm, and sensory feedback to update an estimate of the arm's
% state, as well as this estimate's uncertainty. The estimation is
% performed with an unscented Kalman filter, using the fully nonlinear
% dynamics with state augmented to account for time delay. 
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
%
% The function outputs the estimated state vector for the current time
% step. It also updates state variables for the arm model, which will
% be called upon for both control and estimation during the next time step.

function x_est = estimate(armModel, u, x_sens)

% perform unscented Kalman filtering
[z_est, P] = ukf(armModel, x_sens, u, @actuate, @sense);

% extract estimate of current state from augmented state estimate
nStates = length(armModel.x.val);
x_est = z_est(1:nStates);

% check for direction changes at each joint
nJoints = length(armModel.q.val);
v_estCurr = armModel.x.val(nJoints+1:2*nJoints);
v_estNext = x_est(nJoints+1:2*nJoints);
changedDir = (sign(v_estCurr) ~= sign(v_estNext));

% update state variables for internal model object
armModel.u.val = u;
armModel.x.val = x_est;
armModel.q.val = x_est(1:nJoints);
armModel.q0 = armModel.q0 .* (~changedDir) + armModel.q.val .* changedDir;
[armModel.y.val, armModel.elbw, armModel.inWS] = armModel.fwdKin;
armModel.z.val = z_est;
armModel.P = P;

end