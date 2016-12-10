% This function solves the nonlinear equations of motion of an arm model
% (over a single time step) using MATLAB's variable-step-size numerical
% integrator 'ode45'.
%
% ___________                     _____________________________
% |         |              u*     |                           |   x
% | Control |_____________________| Plant                     |____
% |_________|              |      |   solve the differential  |
%       |                  |      |   equation of motion for  |
%       |                  |      |   the model given by      |
%       |                  |      |   x_dot = f(x,u)          |
%       |                  |      |___________________________|
%       |           _______|_____            | 
%       |           |           |            | x_sens
%       |___________| Estimator |____________|
%         x_est     |___________|
%
%
% The function updates the arm properties and outputs the augmented state
% vector. The augmented state being the sole output of this function is a
% requirement for the function to be used in the unscented Kalman filter
% for state estimation  (see 'estimate.m').
function zNext = plant(arm, u)

% solve equations of motion using ode45, starting from current arm state
% and assuming that joint torques remain constant over the time step
[~, xTraj] = ode45(@(t,x) dynamics(arm,x,u), [0,arm.Ts], arm.x.val);
xNext = xTraj(end,:);

% add motor noise (scaled by time step) to integrated result
nStates = length(arm.x.val);
xNext = xNext + arm.Ts * (arm.motrNoise*ones(1,nStates)) .* rand(1,nStates);

% save noisy new arm state in arm model
nJoints = length(arm.q.val);
arm.x.val = xNext;
arm.q.val = xNext(1:nJoints)';
[arm.y.val, ~, ~] = fwdKin(arm);

% update current state within augmented state vector
zNext = arm.z.val;
zNext(1:nStates) = arm.x.val;

% time shift past states within augmented state vector
nDelay = ceil(arm.Td/arm.Ts);
Mshift = diag(ones(nStates*nDelay,1), -nStates);
zNext = zNext + Mshift*zNext;

% save augmented state in arm model
arm.z.val = zNext;

end