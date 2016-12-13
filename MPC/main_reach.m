close all
clear
clc

% add folders to path
addpath(genpath([pwd '/include']));

% define subject arm (physical)
actSubj.hand = 'right'; % hand being tested
actSubj.M = 70;         % mass [kg]
actSubj.H = 1.80;       % height [meters]
arm = arm_2DOF(actSubj);

% extract arm parameters
nJoints = length(arm.q.val);
nStates = length(arm.x.val);
nInputs = length(arm.u.val);
nOutputs = length(arm.y.val);

% define internal model (mental)
modSubj.hand = 'right';
modSubj.M = 70;
modSubj.H = 1.80;
intModel = arm_2DOF(modSubj);

% update model parameters (e.g., if the subject has suffered a stroke,
% muscle synergies/joint coupling might not be captured by the internal
% model)
stroke = 0;
if stroke
    arm.Td = 0.16;         % increased delay from deafferentation
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
th = 10;                           % reach angle [deg]
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
q = zeros(nJoints,n);
x = zeros(nStates,n);
y = zeros(nOutputs,n);
q_est = zeros(nJoints,n);
x_est = zeros(nStates,n);
y_est = zeros(nOutputs,n);

% simulate reach
progBar = waitbar(0,'Simulating reach ... t = ');
for i = 1:n
    
    % display progress of simulation
    waitbar(i/n, progBar, ['Simulating reach ... t = ',num2str(t(i))]);
    
    % save current data
    u(:,i) = arm.u.val; % = intModel.u.val (they receive same input)
    q(:,i) = arm.q.val;
    x(:,i) = arm.x.val;
    y(:,i) = arm.y.val;
    q_est(:,i) = intModel.q.val;
    x_est(:,i) = intModel.x.val;
    y_est(:,i) = intModel.y.val;
    
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
data.q.act = q;
data.x.act = x;
data.y.act = y;
data.q.est = q_est;
data.x.est = x_est;
data.y.est = y_est; 

% display results of simulation
plotResults(arm, data)
