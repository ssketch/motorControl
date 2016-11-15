function arm = plant( arm, u )
% This function solves the nonlinear equations of motion of a system using
% a variable step size numerical integrator, ode45.m.

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

% Solve the differential equation using ode45
[ ~, y ] = ode45(@(t,q) dynamics(arm, q, u), [0, arm.Ts], arm.q ); 

% Save the integrated result as the new state of the arm.
arm.q = y(end,:)';