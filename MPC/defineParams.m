function p = defineParams(subj, movt, estm, optm)
%% GENERAL

numJoints = subj.numJoints; % number of joint degrees of freedom
p.Nstates = 2*numJoints;    % number of states (th & th_dot for each joint)
p.Nactuat = 1*numJoints;    % number of commands sent to arm (Tcomm for each joint)
p.Nsensed = 2*numJoints;    % number of sensed states (th & th_dot for each joint)

%% SUBJECT

% anthropometry from Winter, 2009
p.m1 = 0.028*subj.M;   % upperarm mass [kg]
p.m2 = 0.022*subj.M;   % forearm mass [kg]
p.l1 = 0.188*subj.H;   % upperarm length [m]
p.l2 = 0.253*subj.H;   % forearm length [m]
p.s1 = 0.436*p.l1;     % shoulder to upperarm COM [m]
p.s2 = 0.682*p.l2;     % elbow to forearm COM [m]
r1 = 0.542*p.l1;       % upperarm radius of gyration (proximal) [m]
r2 = 0.827*p.l2;       % forearm radius of gyration (proximal) [m]
p.I1 = p.m1*r1^2;      % upperarm moment of inertia, about shoulder [kg-m^2]
p.I2 = p.m2*r2^2;      % forearm moment of inertia, about elbow [kg-m^2]
p.B = [0.05 0.025      % damping matrix [Nms/rad]
       0.025 0.05];

% joint limits
toRad = pi/180;
th1Min = -70*toRad; th1Max = 120*toRad;        % shoulder angle limits [rad]
th2Min = 0*toRad;   th2Max = 170*toRad;        % elbow angle limits [rad]
th1dotMin = -50*toRad;  th1dotMax = 90*toRad;  % shoulder velocity limits [rad/s]
th2dotMin = -120*toRad; th2dotMax = 120*toRad; % elbow velocity limits [rad/s]
torq1Min = -85;     torq1Max = 100;            % shoulder torque limits [Nm]
torq2Min = -60;     torq2Max = 75;             % elbow torque limits [Nm]
p.th_lim = [th1Min, th1Max;
            th2Min, th2Max];
p.thdot_lim = [th1dotMin, th1dotMax;
               th2dotMin, th2dotMax];
p.torq_lim = [torq1Min, torq1Max;
              torq2Min, torq2Max];

% neural characteristics
if subj.healthy
    angle1Data = [20;40;65]*toRad;  bias1Data = [-2;0;3]*toRad;
    angle2Data = [30;60;80]*toRad;  bias2Data = [-6;2;7]*toRad;
    [p.biasSlope(1,1), p.biasInter(1,1)] = fitData(angle1Data, bias1Data);
    [p.biasSlope(2,1), p.biasInter(2,1)] = fitData(angle2Data, bias2Data); % bias, as function of joint angle [rad]
    p.noise(1,1) = 0.01*toRad;  p.noise(2,1) = 0.01*toRad;                 % noise standard deviation [rad]
    p.Td = 0.1;                 p.Td_model = p.Td;                         % actual & modeled feedback delays [sec]
else
    angle1Data = [20;40;60]*toRad;  bias1Data = [-10;0;12]*toRad;
    angle2Data = [80;100]*toRad;    bias2Data = [-8;5]*toRad;
    [p.biasSlope(1,1), p.biasInter(1,1)] = fitData(angle1Data, bias1Data);
    [p.biasSlope(2,1), p.biasInter(2,1)] = fitData(angle2Data, bias2Data);
    p.noise(1,1) = 10*toRad;    p.noise(2,1) = 6*toRad;
    %p.Td = 0.16;                p.Td_model = 0.1;
    p.Td = 0.1;                 p.Td_model = 0.1;
end

%% TIMING

% simulation
T = movt.T;        % movement time [sec]
p.dt = movt.dt;    % sampling time [sec]
p.t = 0:p.dt:T;    % time vector [sec]
p.n = length(p.t); % number of time steps

% optimization
Treact = 0.1;              % "reaction time" for replanning torque trajectory (Wagner & Smith, 2008) [sec]
p.Nhrzn = Treact/p.dt + 1; % number of time steps in optimization horizon

% delays
p.numDelSteps = floor(p.Td/p.dt + 1);           % number of time steps in actual feedback delay
p.numDelSteps_mod = floor(p.Td_model/p.dt + 1); % number of time steps in modeled feedback delay

%% MOVEMENT

% initial conditions
p.p_i = movt.p_i;                              % position [m]
v_i = [0;0];                                   % velocity [m/s]
[~,th_i,th_dot_i,~] = invKin(p.p_i,v_i,[0;0],p); % position & velocity [rad]
p.x0 = [th_i;th_dot_i];                        % state

% reference to be tracked
if (movt.type == 0)
    
    d = movt.d;                                    % distance [m]
    th = movt.th;                                  % angle [deg]
    p.p_f = p.p_i + d*[cosd(th);sind(th)];         % desired end position [m]
    v_f = [0;0];                                   % desired end velocity [m/s]
    [~,th_f,th_dot_f,~] = invKin(p.p_f,v_f,[0;0],p);
    p.y_des = repmat([th_f;th_dot_f],1,p.n);
    
elseif (movt.type == 1)
    
    d = movt.d;
    th = movt.th;
    Thold = movt.Thold; % time to hold [sec]
    T = T + Thold;
    p.t = 0:p.dt:T;
    p.n = length(p.t);
    
    p.p_f = p.p_i + d*[cosd(th);sind(th)];
    v_f = [0;0];
    [~,th_f,th_dot_f,~] = invKin(p.p_f,v_f,[0;0],p);
    p.y_des = repmat([th_f;th_dot_f],1,p.n);
    
elseif (movt.type == 2)
    
    d = movt.d;
    th = movt.th;
    T = 2*T;
    p.t = 0:p.dt:T;
    p.n = length(p.t);
    
    p.p_atTurn = p.p_i + d*[cosd(th);sind(th)];
    v_atTurn = [0;0];
    [~,th_atTurn,th_dot_atTurn,~] = invKin(p.p_atTurn,v_atTurn,[0;0],p);
    y_atTurn = [th_atTurn;th_dot_atTurn];
    
    p_f = p.p_i;
    v_f = [0;0];
    [~,th_f,th_dot_f,~] = invKin(p_f,v_f,[0;0],p);
    
    p.y_des = repmat([th_f;th_dot_f],1,p.n);
    p.y_des(:,1:floor(p.n/2)) = repmat(y_atTurn,1,floor(p.n/2));
    
elseif (movt.type == 3)
    
    th = movt.th;
    a = movt.a;
    b = movt.b;
    T = 2*T;
    p.t = 0:p.dt:T;
    p.n = length(p.t);
    
    t_adj = pi + (2*pi/T)*p.t;
    p.x_ell = a*cos(t_adj)*cosd(th) - b*sin(t_adj)*sind(th) + p.p_i(1);
    p.y_ell = a*cos(t_adj)*sind(th) + b*sin(t_adj)*cosd(th) + p.p_i(2);
    
    th_ell = zeros(2,p.n);
    th_dot_ell = zeros(2,p.n);
    for i = 1:p.n
        [~,th_ell(:,i),th_dot_ell(:,i),~] = ...
            invKin([p.x_ell(i);p.y_ell(i)],[0;0],[0;0],p);
    end
    p.y_des = [th_ell;th_dot_ell];
    
end

%% LINEARIZATION

p.C = zeros(p.Nsensed,p.Nstates);
p.C(:,1:p.Nsensed) = eye(p.Nsensed);

% extended state to account for time delay
p.Cext_mod = [zeros(p.Nsensed,p.Nstates*p.numDelSteps_mod) p.C zeros(p.Nsensed,1)]; % modeled time delay
p.Cext_act = [zeros(p.Nsensed,p.Nstates*p.numDelSteps) p.C zeros(p.Nsensed,1)];     % actual time delay

%% FILTERING

q = estm.q;
p.Q = sqrt(p.dt)^2 * diag([q q q q]);              % covariance of process noise, scaled by time step
p.Qext = zeros(p.Nstates*(p.numDelSteps_mod+1)+1); % covariance of process noise, extended for time delay
p.Qext(1:p.Nstates,1:p.Nstates) = p.Q;

rp1 = estm.rp1;              % modeled shoulder joint proprioceptive noise [rad]
rp2 = estm.rp2;              % modeled elbow joint proprioceptive noise [rad]
rv = estm.rv;                % modeled joint velocity proprioceptive noise [rad/s]
p.R = diag([rp1 rp2 rv rv]); % covariance of proprioceptive feedback noise, scaled by time step

%% OPTIMIZATION

% constraints
p.constrainX = 1;
p.constrainU = 1;
%p.xMin = [p.th_lim(:,1);p.thdot_lim(:,1)];
%p.xMax = [p.th_lim(:,2);p.thdot_lim(:,2)];
p.xMin = [p.th_lim(:,1);-inf;-inf];
p.xMax = [p.th_lim(:,2); inf; inf];
p.uMin = p.torq_lim(:,1);
p.uMax = p.torq_lim(:,2);

% costs
wP = optm.wP;
wV = optm.wV;
wT = optm.wT;
alpha = optm.alpha;
p.xCost = alpha*diag([wP;wP;wV;wV]);
p.uCost = diag([wT;wT]);

end