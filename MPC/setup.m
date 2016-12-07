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

% update model parameters
% TO DO

% define movement parameters
movt.space = 'task';       % movement space ('joint' / 'task')
movt.type = 'reach';       % movement type ('reach' / 'hold' / 'outback' / 'ellipse')
movt.T = 1;                % movement time [sec]
movt.t = 0:movt.Ts:movt.T; % time vector [sec]
movt.n = length(movt.t);   % number of time steps
movt.d = 0.35;             % reach distance [m]
movt.th = 10;              % reach angle [deg]
movt.Thold = 2;            % time to hold at endpoint [sec]
movt.a = 0.25;             % ellipse semi-major axis [m]
movt.b = 0.1;              % ellipse semi-minor axis [m]
movt.p_i = [-0.15;0.3];    % initial position [m]
movt.v_i = [0;0];          % initial velocity [m/s]

% define control parameters
Treact = 0.1;                            % "reaction time" for replanning torque trajectory (Wagner & Smith, 2008) [sec]
ctrl.H = Treact/movt.Ts + 1;             % MPC prediction horizon
crtl.wP = 1;                             % position cost
ctrl.wV = 1e-2;                          % velocity cost
ctrl.wU = 1;                             % control effort cost
ctrl.alph = 1e10;                        % weighting between state (pos/vel) and control costs
[ctrl.ref, inWS] = defineRef(arm, movt); % trajectory to track
if ~inWS
    warning('Reference trajectory outside subject workspace.');
end

% define estimation parameters
est.noise = 0;
