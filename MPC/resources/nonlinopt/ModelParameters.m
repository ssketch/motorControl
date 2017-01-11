function params = ModelParameters( M, H )
% Model Parameters
% params = ModelParameters( M, H ) returns the model parameters, params, 
% including the link masses, inertias, joint limits, torque limits, etc. 
% scaled to a subject's input mass, M (kilograms), and height, H (meters), 
% from Winter 2009.
% 
% Note: Segment 1 is the upper arm (humerus) and segment 2 is the forearm
% (radioulnus).

% Anthropometric data
params.M   = M;            % full body mass (kg)
params.H   = H;            % subject height (m)
params.m1  = 0.028 * M;    % upperarm mass [kg]
params.m2  = 0.022 * M;    % forearm mass [kg]
params.L1  = 0.188 * H;    % upperarm length [m]
params.L2  = 0.253 * H;    % forearm length [m]
params.Lc1 = 0.436 * params.L1;   % shoulder to upperarm COM [m]
params.Lc2 = 0.682 * params.L2;   % elbow to forearm COM [m]
params.r1  = 0.542 * params.L1;   % upperarm radius of gyration [m]
params.r2  = 0.827 * params.L2;   % forearm radius of gyration [m]
params.I1  = params.m1 * params.r1^2;    % upperarm moment of inertia about shoulder [kg-m^2]
params.I2  = params.m2 * params.r2^2;    % forearm moment of inertia about elbow [kg-m^2]
b1 = 0.22;       % shoulder damping [Nms/rad]
b2 = 0.22;       % elbow damping [Nms/rad]
toDeg = 180/pi;
params.B = diag([b1 b2]) * (1/toDeg);

% Joint limits
params.th1Min = -70; % shoulder/elbow min/max angles [deg]
params.th1Max = 100;
params.th2Min = 0;
params.th2Max = 160;
params.th1_lim = [ params.th1Min,    params.th1Max ];
params.th2_lim = [ params.th2Min,    params.th2Max ];

% Actuator limits
params.torq1Min = -85; % shoulder/elbow min/max torques [Nm]
params.torq1Max = 100;
params.torq2Min = -60;
params.torq2Max = 75;
params.torq1_lim = [ params.torq1Min,   params.torq1Max ];
params.torq2_lim = [ params.torq2Min,   params.torq2Max ];



% muscle dynamics
params.tau = 0.066; % time constant [sec]