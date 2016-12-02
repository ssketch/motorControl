function [arm, xNext] = plant(arm, x, u)
% This function solves the nonlinear equations of motion of an arm model
% (over a single time step) using MATLAB's variable-step-size numerical
% integrator 'ode45'. It updates both the state attribute of the arm, as
% well as the state vector x augmented to account for time delay.

% ___________                     __________________________________
% |         |                     |                                |
% | Control |_____________________| Plant                          |
% |_________|              |      |    solve the differential      |
%       |                  |      |    equation of motion for the  |
%       |                  |      |    for the model given by      |
%       |                  |      |    x_dot = f(x,u)              |
%       |                  |      |________________________________|
%       |           _______|_____     | 
%       |           |           |     |
%       |___________| Estimator |_____|
%                   |___________|     

% solve the equations of motion using ode45
[~, qTraj] = ode45(@(t,q) dynamics(arm,q,u), [0,arm.Ts], arm.q);

% save the integrated result as new state of the arm
arm.q = qTraj(end,:)';

% save new arm state in the augmented state vector
xNext = x;
xNext(1:arm.nStates) = arm.q;

% time-shift remainder of augmented state vector
nDelSteps = floor(arm.Td/arm.Ts + 1);
Mprop = diag(ones((arm.nStates)*nDelSteps,1),-(arm.nStates)); % time-shift matrix
xNext = xNext + Mprop*xNext;

