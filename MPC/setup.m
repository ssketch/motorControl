% This script provides an example of parameters that must be set prior to
% calling 'plant.m', 'control.m', or 'estimate.m'. Most likely, this code
% will be copied into a block of code at the top of a "main" script.

% define constants
toRad = pi/180;

% define subject parameters (physical)
actSubj.M = 70;                                    % mass [kg]
actSubj.H = 1.80;                                  % height [meters]
actSubj.hand = 'right';                            % handedness
actSubj.thMin = [-70;0]*toRad;                     % joint angle mins [rad]
actSubj.thMax = [120;170]*toRad;                   % joint angle maxs [rad]
actSubj.thdotMin = [-inf;-inf]*toRad;              % joint velocity mins [rad/s]
actSubj.thdotMax = [inf;inf]*toRad;                % joint velocity maxs [rad/s]
actSubj.torqMin = [-85;-60];                       % joint torque mins [Nm]
actSubj.torqMax = [100,75];                        % joint torque maxs [Nm]
actSubj.coupled = 0;                               % 0 = no joint coupling, 1 = coupling present
actSubj.C = NaN;                                   % joint-coupling matrix (only if 'coupled' = 1)
actSubj.Td = 0.16;                                 % time delay [sec]
actSubj.motrNoise = 0.0001;                        % standard deviation of motor noise [Nm]
actSubj.sensNoise = [0.01;0.01;0.001;0.001]*toRad; % standard deviation of sensory noise [rad]
actSubj.biasData(:,:,1) = [20 -2;40 0;65 3]*toRad; % shoulder angle-bias pairs [rad]
actSubj.biasData(:,:,2) = [30 -6;60 2;80 7]*toRad; % elbow angle-bias pairs [rad]
actSubj.sensBias = fitBiasFunc(actSubj.biasData);  % slope & intercept vectors defining sensory bias [rad]

% define internal model parameters (mental)
modSubj.M = 70;
modSubj.H = 1.80;
modSubj.hand = 'right';
modSubj.thMin = [-70;0]*toRad;
modSubj.thMax = [120;170]*toRad;
modSubj.thdotMin = [-inf;-inf];
modSubj.thdotMax = [inf;inf];
modSubj.torqMin = [-85;-60];
modSubj.torqMax = [100,75];
modSubj.coupled = 0;
modSubj.C = NaN;
modSubj.Td = 0.10;
modSubj.motrNoise = 0.0001;
modSubj.sensNoise = [0.0001;0.0001;0.00001;0.00001]*toRad;
modSubj.biasData(:,:,1) = [20 0;40 0;65 0]*toRad;
modSubj.biasData(:,:,2) = [30 0;60 0;80 0]*toRad;
modSubj.sensBias = fitBiasFunc(modSubj.biasData);

% create arm objects
arm = arm_2DOF(actSubj);
intModel = arm_2DOF(modSubj);

% define movement parameters
movt.space = 'task';       % movement space ('joint' / 'task')
movt.type = 'reach';       % movement type ('reach' / 'hold' / 'outback' / 'ellipse')
movt.T = 1;                % movement time [sec]
movt.Ts = 0.01;            % time step [sec]
movt.t = 0:movt.Ts:movt.T; % time vector [sec]
movt.n = length(movt.t);   % number of time steps
movt.d = 0.35;             % reach distance [m]
movt.th = 10;              % reach angle [deg]
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
