close all
clear
clc

% add folders to path
addpath(genpath([pwd '/include']));

% define parameters
debug = 0;
toDeg = 180/pi;
toRad = pi/180;
planar = 1;

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
stroke = 1;
if stroke
    %arm.Td = 0.16;              % increased feedback delay
    arm.coupling = eye(nInputs); % representing muscle synergies
    posNoise = 10;               % (Yousif, 2015)
    arm.sensNoise(1:nJoints) = posNoise*ones(nJoints,1)*toRad;
    biasData_stroke(:,:,1) = [25 -8;35 -2;50 6]*toRad; % (Yousif, 2015)
    biasData_stroke(:,:,2) = [80 -8;90 0;100 6]*toRad;
    arm.sensBias = defineBiasFunc(biasData_stroke);
    intModel.motrNoise = 1e-6; % prediction noise (arbitrary)
end

% define movement
T = 1;                               % total time to simulate [sec]
t = 0:arm.Ts:T;                      % time vector [sec]
n = length(t);                       % number of time steps
d = 0.35;                            % reach distance [m]
th = 10;                             % reach angle [deg]
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
u = zeros(nInputs,n);       % [Nm]
qAct = zeros(nJoints,n);    % [deg]
qEst = zeros(nJoints,n);    % [deg]
xAct = zeros(nStatesJnt,n); % [deg,deg/s]
xEst = zeros(nStatesJnt,n); % [deg,deg/s]
yAct = zeros(nStatesTsk,n); % [m,m/s]
yEst = zeros(nStatesTsk,n); % [m,m/s]

% simulate reach
progBar = waitbar(0,'Simulating reach ... t = ');
for i = 1:n
    
    % display progress of simulation
    waitbar(i/n, progBar, ['Simulating reach ... t = ',num2str(t(i))]);
    if debug
        draw(arm);
        disp(' ');
        disp('x = ');
        disp(arm.x.val);
        disp(' ');
        disp('x_est = ');
        disp(intModel.x.val);
    end
    
    % save current data
    u(:,i) = arm.u.val; % = intModel.u.val (they receive same input)
    qAct(:,i) = arm.q.val*toDeg;
    qEst(:,i) = intModel.q.val*toDeg;
    xAct(:,i) = arm.x.val*toDeg;
    xEst(:,i) = intModel.x.val*toDeg;
    yAct(:,i) = arm.y.val;
    yEst(:,i) = intModel.y.val;
    
    % compute optimal control trajectory (only if enough time has passed)
    if mod(t(i),arm.Tr) == 0
        [u_optTraj, flag] = control(intModel, ref(:,i), space);
        if flag
            warning('Linearization failed.')
            return
        end
    end
    
    % set torque to zero if haven't planned that far in advance;
    % otherwise, grab torque from preplanned trajectory
    if isempty(u_optTraj)
        u_opt = zeros(nInputs,1);
    else
        u_opt = u_optTraj(:,1);
    end
    
    % actuate arm with optimal control & sense feedback
    zNext = plant(arm, u_opt);
    x_sens = sense(arm, zNext);
    
    % estimate current state
    x_est = estimate(intModel, u_opt, x_sens);

    % discard most recently applied control
    u_optTraj = u_optTraj(:,2:end);

end
close(progBar)

% save data in a struct
data.t = t;
data.u = u;
data.qAct = qAct;
data.qEst = qEst;
data.xAct = xAct;
data.xEst = xEst;
data.yAct = yAct;
data.yEst = yEst;

% display results of simulation
plotResults(arm, data, planar)
