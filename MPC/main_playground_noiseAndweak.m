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
nTrials = 5;                 % number of times to repeat reach
T = 1;                       % total time to simulate [sec]
movt.t = 0:arm.Ts:T;         % time vector [sec]
r = 0.15;                    % reach distance [m]
th = 135;                    % reach angles [deg]
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

% loop over deficits
for i = 1:2
    
    % reseed random number generator
    rng(1)
    
    % reinitialize models
    arm = arm_2DOF(subj);
    intModel = arm_2DOF(subj);
    
    % dealing with prediction error
    if i == 1
        
        % define sensory noise & bias
        toRad = pi/180;
        posNoise = 10;
        arm.sensNoise(1:nJoints) = posNoise*ones(nJoints,1)*toRad;
        biasData_stroke(:,:,1) = [25 -8;35 -2;50 6]*toRad;
        biasData_stroke(:,:,2) = [80 -8;90 0;100 6]*toRad;
        arm.sensBias = defineBiasFunc(biasData_stroke);
        
        % loop over prediction noise values
        noise = [0.2;2;8];
        for n = 1:length(noise)
            
            % assign prediction noise to internal model
            intModel.motrNoise = noise(n);
            
            % loop over trials
            for j = 1:nTrials
                
                % reset model state variables to match initial conditions for movement
                arm.u.val = zeros(nInputs,1);
                arm.x.val = [x_i;zeros(nInputs,1)];
                arm.q.val = x_i(1:nJoints);
                arm.y.val = arm.fwdKin;
                nDelay = ceil(arm.Td/arm.Ts);
                arm.z.val = repmat(arm.x.val, nDelay+1, 1);
                arm.P = diag(1e-6*ones(length(arm.z.val),1));
                
                intModel.u.val = zeros(nInputs,1);
                intModel.x.val = [x_i;zeros(nInputs,1)];
                intModel.q.val = x_i(1:nJoints);
                intModel.y.val = intModel.fwdKin;
                nDelay = ceil(intModel.Td/intModel.Ts);
                intModel.z.val = repmat(intModel.x.val, nDelay+1, 1);
                intModel.P = diag(1e-6*ones(length(intModel.z.val),1));
                
                % simulate reach
                data = simulate(movt, arm, intModel);
                u = data.uCmd;
                x = data.xAct;
                y = data.yAct;
                
                % save data
                filename = ['./results/pub3/reach',num2str(th),...
                    '_stroke_predErr_n',num2str(intModel.motrNoise),...
                    '_',num2str(j),'.mat'];
                save(filename,'u','x','y');
                
            end
            
        end
        
    % dealing with muscular weakness
    else
        
        % loop over strength values
        strength = [0.8;0.4;0.1];
        for s = 1:length(strength)
            
            % assign strength to internal model
            intModel.strength = strength(s);
            
            % loop over trials
            for j = 1:nTrials
                
                % reset model state variables to match initial conditions for movement
                arm.u.val = zeros(nInputs,1);
                arm.x.val = [x_i;zeros(nInputs,1)];
                arm.q.val = x_i(1:nJoints);
                arm.y.val = arm.fwdKin;
                nDelay = ceil(arm.Td/arm.Ts);
                arm.z.val = repmat(arm.x.val, nDelay+1, 1);
                arm.P = diag(1e-6*ones(length(arm.z.val),1));
                
                intModel.u.val = zeros(nInputs,1);
                intModel.x.val = [x_i;zeros(nInputs,1)];
                intModel.q.val = x_i(1:nJoints);
                intModel.y.val = intModel.fwdKin;
                nDelay = ceil(intModel.Td/intModel.Ts);
                intModel.z.val = repmat(intModel.x.val, nDelay+1, 1);
                intModel.P = diag(1e-6*ones(length(intModel.z.val),1));
                
                % simulate reach
                data = simulate(movt, arm, intModel);
                u = data.uCmd;
                x = data.xAct;
                y = data.yAct;
                
                % save data
                filename = ['./results/pub3/reach',num2str(th),...
                    '_stroke_weak_c',num2str(intModel.strength),...
                    '_',num2str(j),'.mat'];
                save(filename,'u','x','y');
                
            end
        end
        
    end
    
end
