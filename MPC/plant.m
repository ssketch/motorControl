function zNext = plant(arm)
% This function solves the nonlinear equations of motion of an arm model
% (over a single time step) using MATLAB's variable-step-size numerical
% integrator 'ode45'. It updates the arm properties and outputs the
% augmented state vector (as required for this function to be used in the
% unscented Kalman filter for state estimation).
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
%       |           |           |            | y
%       |___________| Estimator |____________|
%         x_est     |___________|

% solve the equations of motion using ode45, starting from the current arm
% state and assuming that joint torques remain constant over the time step
[~, xTraj] = ode45(@(t,x) dynamics(arm,x), [0,arm.Ts], arm.x.val);

% save the integrated result as new state of the arm
nJnts = length(arm.q.val);
arm.q.val = xTraj(end,1:nJnts)';
arm.x.val = xTraj(end,:)';

% save new arm state in the augmented state vector
nStates = length( arm.x.min );
zNext = arm.x.val;
zNext(1:nStates) = arm.x.val;

% time-shift remainder of augmented state vector
nDelSteps = floor(arm.Td/arm.Ts + 1);
Mprop = diag(ones((nStates)*nDelSteps,1),-(nStates)); % time-shift matrix
xNext = xNext + Mprop*xNext;

if ~model
    xNext(1:nStates) = xNext(1:nStates) + ...
        sqrt(diag(params.Q)).*randn(nStates,1);
end