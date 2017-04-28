close all
clear
clc

% add folders to path
addpath(genpath([pwd '/include']));

% define subject
subj.hand = 'right'; % hand being tested
subj.M = 70;         % mass [kg]
subj.H = 1.80;       % height [meters]
predErr = 0;         % 1 = stroke caused prediction error
synerg = 0;          % 1 = stroke coupled muscle synergies
weak = 0;            % 1 = stroke caused muscular weakness
spastic = 1;         % 1 = stroke caused spasticity/increased tone

% define subject's physical arm & internal arm model (mental)
arm = arm_2DOF(subj);
intModel = arm_2DOF(subj);

% reduce reaction time for feasibility with linear optimization & nonlinear
% model
Tr_red = 0.03;
arm.Tr = Tr_red;
intModel.Tr = Tr_red;

% extract arm parameters
nInputs = length(arm.u.val);
nJoints = length(arm.q.val);

% update model parameters for stroke-induced deficits
if predErr
    % from (Yousif, 2015), for deafferented patient
    toRad = pi/180;
    posNoise = 10;
    arm.sensNoise(1:nJoints) = posNoise*ones(nJoints,1)*toRad;
    biasData_stroke(:,:,1) = [25 -8;35 -2;50 6]*toRad;
    biasData_stroke(:,:,2) = [80 -8;90 0;100 6]*toRad;
    arm.sensBias = defineBiasFunc(biasData_stroke);
    
    % prediction noise
    intModel.motrNoise = 2;
end
if synerg
    % from (Dewald, 1995), representing muscle synergies induced by stroke
    Msynerg = [1, 0, 0, 0; 0, 1, 0.13, 0.63;
               0, 0, 1, 0; 0.8, 0.06, 0, 1];
    for i = 1:size(Msynerg,1)
        Msynerg(i,:) = Msynerg(i,:) / sum(Msynerg(i,:));
    end
    arm.coupling = arm.coupling * Msynerg;
end
if weak
    % arbitrary scalar for control limits in MPC
    % (NOTE: imposed on internal model because it is used for control)
    intModel.strength = 0.8;
end
if spastic
    % from (Levin & Feldman, 2003), (McCrea, 2003), & (Given, 1995)
    % NOTE: this is for the shoulder and elbow, assuming the following:
    % ----  (1) flexion/extension are equivalent, (2) mu does not depend
    %       on MAS, (3) shoulder is equivalent to elbow with a 10x increase
    %       in reflex stiffness
    toRad = pi/180;
    spasticData(:,:,2) = [[-5.70    73.37]*toRad;               % gamma [rad]
                          [0        0.25];                      % mu [sec]
                          [2.22e-4  1.62e-4]*(1/toRad)*subj.M;  % k [Nm/rad]
                          [7.12e-5  2.47e-5]*(1/toRad)*subj.M]; % b [Nms/rad]
    spasticData(:,:,1) = spasticData(:,:,2);
    spasticData(3,:,1) = spasticData(3,:,1)*10; % shoulder = 10x stiffer
    
    MAS = 4; % score on Modified Ashworth scale [0,1,1.5(1+),2,3,4]
    [arm.gamma, arm.mu, arm.k, arm.b] = ...
        defineSpasticity(MAS, spasticData);
end

% define movement parameters
nTrials = 1;                 % number of times to repeat reach
T = 0.75;                    % total time to simulate [sec]
movt.t = 0:arm.Ts:T;         % time vector [sec]
r = 0.4;                     % reach distance [m]
th = 115;                    % reach angles [deg]
origin1 = [-0.15;0.3;0];     % origin 1 (arbitrary) [m]
origin2 = [-0.18;0.56;0];    % origin 2, to match (Beer, 2000) [m]
origin3 = [-0.15;0.6;0];     % origin 3 (less arbitrary) [m]
p_i = origin1;               % initial position [m]
v_i = [0;0;0];               % initial velocity [m/s]
y_i = [p_i;v_i];             % initial state, in Cartesian coordinates [m,m/s]
[x_i,~,~] = arm.invKin(y_i); % initial state, in joint coordinates [rad,rad/s]
movt.space = 'task';         % space in which to track reference ('joint' or 'task')

% define plotting parameters
plotOn = 1;
    
for n = 1:nTrials
    
    for i = 1:length(th)
        
        % define movement reference trajectory
        p_f = p_i + r*[cosd(th(i));sind(th(i));0]; % desired end position [m]
        v_f = [0;0;0];                             % desired end velocity [m/s]
        y_f = [p_f;v_f];                           % desired end state, in Cartesian coordinates [m,m/s]
        [x_f,~,~] = arm.invKin(y_f);               % desired end state, in joint coordinates [rad,rad/s]
        switch movt.space
            case 'joint'
                movt.ref = repmat(x_f,1,length(movt.t)); % joint-space reference to track [rad,rad/s]
            case 'task'
                movt.ref = repmat(y_f,1,length(movt.t)); % task-space reference to track [m,m/s]
            otherwise
                movt.space = 'task';
                movt.ref = repmat(y_f,1,length(movt.t)); % task space by default
        end
        
        % reset model state variables to match initial conditions for movement
        % NOTE: internal model's state estimates are grounded by vision (i.e.,
        % ----  assuming perfect vision, they match the arm's actual state)
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
        
        % save data
        if synerg
            filename = ['./results/pub2/reach',num2str(th(i)),...
                '_stroke_synerg_',num2str(n),'.mat'];
        elseif predErr
            filename = ['./results/pub2/reach',num2str(th(i)),...
                '_stroke_predErr_n',num2str(intModel.motrNoise),...
                '_',num2str(n),'.mat'];
        elseif weak
            filename = ['./results/pub2/reach',num2str(th(i)),...
                '_stroke_weak_c',num2str(intModel.strength),...
                '_',num2str(n),'.mat'];
        elseif spastic
            filename = ['./results/pub2/reach',num2str(th(i)),...
                '_stroke_spastic_MAS',num2str(MAS),...
                '_',num2str(n),'.mat'];
        else
            filename = ['./results/pub2/reach',num2str(th(i)),'_ctrl.mat'];
        end
        u = data.uCmd;
        x = data.xAct;
        y = data.yAct;
        save(filename,'u','x','y');
        
        % display results of simulation
        if plotOn
            plotResults(arm, data, 1)
        end
        
    end
end
