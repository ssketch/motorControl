function arm = plant(arm, u)
% This function solves the nonlinear equations of motion of an arm model
% (over a single time step) using MATLAB's variable-step-size numerical
% integrator 'ode45'.

% ____________                    __________________________________
% |          |                    |                                |
% | Control  |____________________| Plant                          |
% |__________|             |      |    solve the differential      |
%       |                  |      |    equation of motion for the  |
%       |                  |      |    for the model given by      |
%       |                  |      |    x_dot = f(x,u)              |
%       |                  |      |________________________________|
%       |           _______|______    |
%       |           |            |    |
%       |___________|  Estimator |____|
%                   |____________|     

% solve the equations of motion using ode45
[~, y] = ode45(@(t,q) arm.dynamics(q,u), [0,arm.Ts], arm.q); 

% save the integrated result as new state of the arm
arm.q = y(end,:)';