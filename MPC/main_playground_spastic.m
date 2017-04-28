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
nInputs = length(arm.u.val);
nJoints = length(arm.q.val);

% define movement parameters
nTrials = 1;                 % number of times to repeat reach
T = 1;                       % total time to simulate [sec]
movt.t = 0:arm.Ts:T;         % time vector [sec]
r = 0.35;                    % reach distance [m]
th = 120;                    % reach angles [deg]
p_i = [-0.15;0.3;0];         % initial position [m]
v_i = [0;0;0];               % initial velocity [m/s]
y_i = [p_i;v_i];             % initial state, in Cartesian coordinates [m,m/s]
[x_i,~,~] = arm.invKin(y_i); % initial state, in joint coordinates [rad,rad/s]
movt.space = 'task';         % space in which to track reference ('joint' or 'task')

% define movement reference trajectory
p_f = p_i + r*[cosd(th);sind(th);0]; % desired end position [m]
v_f = [0;0;0];                       % desired end velocity [m/s]
y_f = [p_f;v_f];                     % desired end state, in Cartesian coordinates [m,m/s]
[x_f,~,~] = arm.invKin(y_f);         % desired end state, in joint coordinates [rad,rad/s]
switch movt.space
    case 'joint'
        movt.ref = repmat(x_f,1,length(movt.t)); % joint-space reference to track [rad,rad/s]
    case 'task'
        movt.ref = repmat(y_f,1,length(movt.t)); % task-space reference to track [m,m/s]
    otherwise
        movt.space = 'task';
        movt.ref = repmat(y_f,1,length(movt.t)); % task space by default
end

% define spasticity data
toRad = pi/180;
spasticData(:,:,2) = [[-5.70    73.37]*toRad;               % gamma [rad] (Levin & Feldman, 2003)
                      [0        0.25];                      % mu [sec] (Levin & Feldman, 2003)
                      [2.22e-4  1.62e-4]*(1/toRad)*subj.M;  % k [Nm/rad] (McCrea, 2003)
                      [7.12e-5  2.47e-5]*(1/toRad)*subj.M]; % b [Nms/rad] (McCrea, 2003)
spasticData(:,:,1) = spasticData(:,:,2);
spasticData(3,:,1) = spasticData(3,:,1)*10; % shoulder = 10x stiffer (Given, 1995)

% loop over spasticity scores
MAS = [0;1;1.5;2;3;4];
for n = 1:length(MAS)
    
    % assign spasticity parameters to arm
    [arm.gamma, arm.mu, arm.k, arm.b] = ...
        defineSpasticity(MAS(n), spasticData);
    
    % loop over trials
    for i = 1:nTrials
        
        % reset model state variables to match initial conditions for movement
        arm.u.val = zeros(nInputs,1);
        arm.uReflex = zeros(nJoints,1);
        arm.x.val = [x_i;zeros(nInputs,1)];
        arm.q.val = x_i(1:nJoints);
        arm.q0 = arm.q.val;
        arm.y.val = arm.fwdKin;
        nDelay = ceil(arm.Td/arm.Ts);
        arm.z.val = repmat(arm.x.val, nDelay+1, 1);
        arm.P = diag(1e-6*ones(length(arm.z.val),1));
        
        intModel.u.val = zeros(nInputs,1);
        intModel.uReflex = zeros(nJoints,1);
        intModel.x.val = [x_i;zeros(nInputs,1)];
        intModel.q.val = x_i(1:nJoints);
        intModel.q0 = intModel.q.val;
        intModel.y.val = intModel.fwdKin;
        nDelay = ceil(intModel.Td/intModel.Ts);
        intModel.z.val = repmat(intModel.x.val, nDelay+1, 1);
        intModel.P = diag(1e-6*ones(length(intModel.z.val),1));
        
        % simulate reach
        data = simulate(movt, arm, intModel);
        u = data.uCmd;
        x = data.xAct;
        y = data.yAct;
        reflex = data.uRfx;
        J = data.JAct;
        
        % save data
        filename = ['./results/pub4/reach',num2str(th),...
            '_stroke_spastic_MAS',num2str(MAS(n)),...
            '_',num2str(i),'.mat'];
        save(filename,'u','x','y','reflex','J');
        
    end
end
