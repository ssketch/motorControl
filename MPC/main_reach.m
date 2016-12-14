close all
clear
clc

% add folders to path
addpath(genpath([pwd '/include']));

% define subject
subj.hand = 'right'; % hand being tested
subj.M = 70;         % mass [kg]
subj.H = 1.80;       % height [meters]

% define subject's physical arm & internal arm model (mental)
arm = arm_2DOF(subj);
intModel = arm_2DOF(subj);

% extract arm parameters
nJoints = length(arm.q.val);
nStates = length(arm.x.val);
nInputs = length(arm.u.val);
nOutputs = length(arm.y.val);

% update model parameters (e.g., if the subject has suffered a stroke,
% muscle synergies/joint coupling might not be captured by the internal
% model)
stroke = 0;
if stroke
    arm.Td = 0.16;         % increased delay (e.g., from deafferentation)
    arm.coupling = eye(2); % representing muscle synergies
    posNoise = 10;
    velNoise = 0.1;
    arm.sensNoise = [posNoise; posNoise; velNoise; velNoise]*toRad; % (Cusmano, 2014)
    biasData(:,:,1) = [20 -10;40 0;65 12]*toRad;                    % (Cusmano, 2014)
    biasData(:,:,2) = [80 -8;100 5]*toRad;
    arm.sensBias = defineBiasFunc(biasData);
    intModel.motrNoise = 0.1; % prediction noise
end

% define reach movement
T = 1;                             % total time to simulate [sec]
t = 0:arm.Ts:T;                    % time vector [sec]
n = length(t);                     % number of time steps
d = 0.35;                          % reach distance [m]
th = 30;                           % reach angle [deg]
p_i = [-0.15;0.3];                 % initial position [m]
v_i = [0;0];                       % initial velocity [m/s]
y_i = [p_i;v_i];                   % initial state [m,m/s]
p_f = p_i + d*[cosd(th);sind(th)]; % desired end position [m]
v_f = [0;0];                       % desired end velocity [m/s]
y_f = [p_f;v_f];                   % desired end state [m,m/s]
ref.space = 'task';                % space in which to track reference
ref.traj = repmat(y_f,1,n);        % reference trajectory [m,m/s]

% update model state variables to match initial conditions for movement
% NOTE: internal model's state estimate is grounded by vision (i.e.,
% ----  assuming perfect vision, it matches the arm's actual state)
arm.y.val = y_i;
[arm.x.val, arm.elbw, arm.inWS] = arm.invKin;
arm.q.val = arm.x.val(1:nJoints);
nDelay = ceil(arm.Td/arm.Ts);
arm.z.val = repmat(arm.x.val, nDelay+1, 1);

intModel.y.val = arm.y.val;
[intModel.x.val, intModel.elbw, intModel.inWS] = intModel.invKin;
intModel.q.val = intModel.x.val(1:nJoints);
nDelay = ceil(intModel.Td/intModel.Ts);
intModel.z.val = repmat(intModel.x.val, nDelay+1, 1);

% declare variables to save
u = zeros(nInputs,n);
qAct = zeros(nJoints,n);
qEst = zeros(nJoints,n);
xAct = zeros(nStates,n);
xEst = zeros(nStates,n);
yAct = zeros(nOutputs,n);
yEst = zeros(nOutputs,n);

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
    [u_opt, flag] = control(intModel, t(i), ref.traj(:,i), ref.space);
    
    % check for problems with control computation
    if flag == 1
        warning('Linearization of model failed.')
        return
    elseif flag == 2
        warning('Computed control out of bounds.')
        return
    end
    
    % actuate arm with optimal control & sense feedback
    zNext = plant(arm, u_opt);
    x_sens = sense(arm, zNext);
    
    % estimate current state
    x_est = estimate(intModel, u_opt, x_sens);

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
plotResults(arm, data)
