function xNext = plant(arm, u)
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
[~, xTraj] = ode45(@(t,q) dynamics(arm,q,u), [0,arm.Ts], arm.x.val);

% save the integrated result as new state of the arm
arm.x.val = xTraj(end,:)';

% save new arm state in the augmented state vector
nStates = length( arm.x.min );
xNext = arm.x.val;
xNext(1:nStates) = arm.x.val;

% time-shift remainder of augmented state vector
nDelSteps = floor(arm.Td/arm.Ts + 1);
Mprop = diag(ones((nStates)*nDelSteps,1),-(nStates)); % time-shift matrix
xNext = xNext + Mprop*xNext;

if ~model
    xNext(1:nStates) = xNext(1:nStates) + ...
        sqrt(diag(params.Q)).*randn(nStates,1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function currently not working. My error message:
% Error using  * 
% Inner matrix dimensions must agree.
% 
% Error in plant (line 34)
% xNext = xNext + Mprop*xNext;
% 
% Error in main_workspace (line 84)
%         model = plant( model, u_star );
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
