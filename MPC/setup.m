% This script provides an example of parameters that must be set prior to
% calling 'plant.m', 'control.m', or 'estimate.m'. Most likely, this code
% will be copied into a block of code at the top of a "main" script.

% define subject parameters (physical)
subj.M = 70;         % mass [kg]
subj.H = 1.80;       % height [meters]
subj.Td = 0.16;      % time delay [sec]
subj.coupled = 0;    % 0 = no joint coupling, 1 = coupling present
subj.C = NaN;        % joint-coupling matrix (only if 'coupled' = 1)
subj.hand = 'right'; % handedness

% define internal model parameters (mental)
IM.M = 70;
IM.H = 1.80;
IM.Td = 0.10;
IM.coupled = 0;
IM.C = NaN;
IM.hand = 'right';

% define MPC parameters
ctrl.H = 15;     % prediction horizon

% define UKF parameters
est.