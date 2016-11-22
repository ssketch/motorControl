function arm = plant(arm, u)
% This function solves the nonlinear equations of motion of an arm model
% (over a single time step) using the variable-step-size numerical
% integrator 'ode45'.

% ____________                    __________________________________
% |          |                    |                                |
% | Control  |____________________| Plant: solve the differential  |
% |__________|             |      |    equation of motion for the  |
%       |                  |      |    model given by              |
%       |                  |      |     x_dot = f(x,u)             |
%       |                  |      |________________________________|
%       |           _______|______    |
%       |           |            |    |
%       |___________|  Estimator |____|
%                   |____________|     

% solve the equations of motion using ode45
[ ~, y ] = ode45(@(t,q) dynamics(arm,q,u), [0,arm.Ts], arm.q); 

% save the integrated result as the new state of the arm
arm.q = y(end,:)';