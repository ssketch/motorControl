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
% The function updates the augmented state vector, from which all other
% state variable can be extracted. The augmented state being the sole
% output of this function is a requirement for the function to be used in
% the unscented Kalman filter for state estimation  (see 'estimate.m').
% Note that input 'z' is optional. If 'z' is omitted, the plant will be
% actuated from the current state of the arm object.
function zNext = plant(arm, u, zCurr)

% if no state is specified, use current arm state
if nargin < 3
    zCurr = arm.z.val;
end

% extract current state from augmented state vector
nStates = length(arm.x.val);
xCurr = zCurr(1:nStates);

% solve equations of motion using ode45, starting from current arm state
% and assuming that joint torques remain constant over the time step
[~, xTraj] = ode45(@(t,x) dynamics(arm,x,u), [0,arm.Ts], xCurr);
xNext = xTraj(end,:);

% add motor noise (scaled by time step) to integrated result
xNext = xNext + arm.Ts * (arm.motrNoise*ones(1,nStates)) .* rand(1,nStates);

% update current state within augmented state vector
zNext = zCurr;
zNext(1:nStates) = xNext;

% time shift past states within augmented state vector
nDelay = ceil(arm.Td/arm.Ts);
Mshift = diag(ones(nStates*nDelay,1), -nStates);
zNext = zNext + Mshift*zNext;

% update state variables for arm object
nJoints = length(arm.q.val);
arm.u.val = u;
arm.x.val = xNext;
arm.q.val = xNext(1:nJoints)';
[arm.y.val, arm.elbow, ~] = fwdKin(arm);
arm.z.val = zNext;

end