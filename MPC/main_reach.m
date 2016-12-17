close all
clear
clc

% add folders to path
addpath(genpath([pwd '/include']));

% set optimization algorithm
options = optimset('Algorithm','interior-point');
% SET THIS WHEN 'fmin___' IS CALLED

% define subject
subj.hand = 'right'; % hand being tested
subj.M = 70;         % mass [kg]
subj.H = 1.80;       % height [meters]

% define subject's physical arm & internal arm model (mental)
arm = arm_2DOF(subj);
intModel = arm_2DOF(subj);

% extract arm parameters
nJoints = length(arm.q.val);
nStatesJnt = length(arm.x.val);
nStatesTsk = length(arm.y.val);
nInputs = length(arm.u.val);

% update model parameters (e.g., if the subject has suffered a stroke,
% muscle synergies/joint coupling might not be captured by the internal
% model)
stroke = 0;
if stroke
    arm.Td = 0.16;         % increased feedback delay
    arm.coupling = eye(2); % representing muscle synergies
    posNoise = 10;         % NEED SOURCE
    velNoise = 0.1;        % NEED SOURCE
    arm.sensNoise = [posNoise; posNoise; velNoise; velNoise]*toRad; % (Cusmano, 2014)
    biasData(:,:,1) = [20 -10;40 0;65 12]*toRad;                    % (Cusmano, 2014)
    biasData(:,:,2) = [80 -8;100 5]*toRad;
    arm.sensBias = defineBiasFunc(biasData);
    intModel.motrNoise = 0.1; % prediction noise (arbitrary)
end

% define movement
T = 1;                               % total time to simulate [sec]
t = 0:arm.Ts:T;                      % time vector [sec]
n = length(t);                       % number of time steps
d = 0.35;                            % reach distance [m]
th = 30;                             % reach angle [deg]
p_i = [-0.15;0.3;0];                 % initial position [m]
v_i = [0;0;0];                       % initial velocity [m/s]
y_i = [p_i;v_i];                     % initial state, in Cartesian coordinates [m,m/s]
[x_i,~,~] = arm.invKin(y_i);         % initial state, in joint coordinates [rad,rad/s]
p_f = p_i + d*[cosd(th);sind(th);0]; % desired end position [m]
v_f = [0;0;0];                       % desired end velocity [m/s]
y_f = [p_f;v_f];                     % desired end state, in Cartesian coordinates [m,m/s]
[x_f,~,~] = arm.invKin(y_f);         % desired end state, in joint coordinates [rad,rad/s]
space = 'task';                      % space in which to track reference ('joint' or 'task')
switch space
    case 'joint' 
        ref = repmat(x_f,1,n); % joint-space reference to track [rad,rad/s]
    case 'task'
        ref = repmat(y_f,1,n); % task-space reference to track [m,m/s]
    otherwise
        space = 'task';
        ref = repmat(y_f,1,n); % task space by default
end

% update model state variables to match initial conditions for movement
% NOTE: internal model's state estimates are grounded by vision (i.e.,
% ----  assuming perfect vision, they match the arm's actual state)
arm.x.val = x_i;
arm.q.val = x_i(1:nJoints);
arm.y.val = y_i;
nDelay = ceil(arm.Td/arm.Ts);
arm.z.val = repmat(arm.x.val, nDelay+1, 1);

intModel.x.val = x_i;
intModel.q.val = x_i(1:nJoints);
intModel.y.val = y_i;
nDelay = ceil(intModel.Td/intModel.Ts);
intModel.z.val = repmat(intModel.x.val, nDelay+1, 1);

% declare variables to save
u = zeros(nInputs,n);
qAct = zeros(nJoints,n);
qEst = zeros(nJoints,n);
xAct = zeros(nStatesJnt,n);
xEst = zeros(nStatesJnt,n);
yAct = zeros(nStatesTsk,n);
yEst = zeros(nStatesTsk,n);

% simulate reach
progBar = waitbar(0,'Simulating reach ... t = ');
for i = 1:n
    
    % display progress of simulation
    waitbar(i/n, progBar, ['Simulating reach ... t = ',num2str(t(i))]);
    
    % save current data
    u(:,i) = arm.u.val; % = intModel.u.val (they receive same input)
    qAct(:,i) = arm.q.val;
    xAct(:,i) = arm.x.val;
    yAct(:,i) = arm.y.val;
    qEst(:,i) = intModel.q.val;
    xEst(:,i) = intModel.x.val;
    yEst(:,i) = intModel.y.val;
    
    % compute optimal control
    [u_opt, flag] = control(intModel, t(i), ref(:,i), space);
    if flag
        warning('Linearization failed.')
        return
    end
    
    % actuate arm with optimal control & sense feedback
    zNext = plant(arm, u_opt);
    x_sens = sense(arm, zNext);
    
    % estimate current state
    x_est = estimate(intModel, u_opt, x_sens);

    arm.draw
end
close(progBar)

% save data in a struct
data.t = t;
data.u = u;
data.q.act = qAct;
data.x.act = xAct;
data.y.act = yAct;
data.q.est = qEst;
data.x.est = xEst;
data.y.est = yEst; 

% display results of simulation
%plotResults(arm, data)
