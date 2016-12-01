function arm = plant(arm, u)
% This function solves the nonlinear equations of motion of an arm model
% (over a single time step) using MATLAB's variable-step-size numerical
% integrator 'ode45'.

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