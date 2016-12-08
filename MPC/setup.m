% This script provides an example of parameters that must be set prior to
% calling 'plant.m', 'control.m', or 'estimate.m'. Most likely, this code
% will be copied into a block of code at the top of a "main" script.

% define subject arm (physical)
actSubj.hand = 'right'; % hand being tested
actSubj.M = 70;         % mass [kg]
actSubj.H = 1.80;       % height [meters]
arm = arm_2DOF(actSubj);

% define internal model (mental)
modSubj.hand = 'right';
modSubj.M = 70;
modSubj.H = 1.80;
intModel = arm_2DOF(modSubj);

% update model parameters (e.g., if the subject has suffered a stroke,
% muscle synergies/joint coupling might not be captured by the internal
% model)

%%%%%%%%%
% TO DO %
%%%%%%%%%

% define control parameters
ctrl.H = arm.T/arm.Ts + 1; % MPC prediction horizon
crtl.wP = 1;               % position cost
ctrl.wV = 1e-2;            % velocity cost
ctrl.wU = 1;               % control effort cost
ctrl.alph = 1e10;          % weighting between state (pos/vel) and control costs
ctrl.ref = [1;1];          % reference trajectory

% define estimation parameters
est.noise = 0;
