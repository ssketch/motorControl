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
nReach = 8;                  % total number of (evenly spaced) center-out reaches
nTrials = 5;                 % number of times to repeat 'nReach' trials
T = 1;                       % total time to simulate, for each reach [sec]
movt.t = 0:arm.Ts:T;         % time vector [sec]
r = 0.15;                    % reach distance [m]
thStep = 360/nReach;         % step from one reach angle to next [deg]
th = 0:thStep:360-thStep;    % reach angles [deg]
p_i = [-0.15;0.3;0];         % initial position [m]
v_i = [0;0;0];               % initial velocity [m/s]
y_i = [p_i;v_i];             % initial state, in Cartesian coordinates [m,m/s]
[x_i,~,~] = arm.invKin(y_i); % initial state, in joint coordinates [rad,rad/s]
movt.space = 'task';         % space in which to track reference ('joint' or 'task')

% loop over trials
for n = 5
    
    % just record control data for one trial
    if n == 1
        vec = [0;1];
    else
        vec = 1;
    end
    
    for stroke = 1
        
        % reseed random number generator
        rng(10*n)
        
        % if stroke, couple synergies
        if stroke
            Msynerg = [1, 0, 0, 0; 0, 1, 0.13, 0.63;
                       0, 0, 1, 0; 0.8, 0.06, 0, 1]; % representing muscle synergies (Dewald, 1995)
            for i = 1:size(Msynerg,1)
                Msynerg(i,:) = Msynerg(i,:) / sum(Msynerg(i,:)); % normalization
            end
            arm.coupling = arm.coupling * Msynerg;
        end
        
        % loop over circle of reaches
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
            
            % update model state variables to match initial conditions for movement
            arm.x.val = [x_i;zeros(nInputs,1)];
            arm.q.val = x_i(1:nJoints);
            arm.y.val = arm.fwdKin;
            nDelay = ceil(arm.Td/arm.Ts);
            arm.z.val = repmat(arm.x.val, nDelay+1, 1);
            arm.P = diag(1e-6*ones(length(arm.z.val),1));
            
            intModel.x.val = [x_i;zeros(nInputs,1)];
            intModel.q.val = x_i(1:nJoints);
            intModel.y.val = intModel.fwdKin;
            nDelay = ceil(intModel.Td/intModel.Ts);
            intModel.z.val = repmat(intModel.x.val, nDelay+1, 1);
            intModel.P = diag(1e-6*ones(length(intModel.z.val),1));
            
            % simulate reach
            data = simulate(movt, arm, intModel);
            
            % save state trajectories
            U(:,:,i) = data.uCmd;
            X(:,:,i) = data.xAct;
            Y(:,:,i) = data.yAct;
            
        end
        
        % save all trajectories into MAT file
        if stroke
            filename = ['./results/pub3/circle_stroke_synerg_',num2str(n),'.mat'];
        else
            filename = ['./results/pub3/circle_ctrl.mat'];
        end
        save(filename,'U','X','Y');
        
        % if 8 reaches, save specific trajectories into MAT files
        if nReach == 8
            if stroke
                filename45 = ['./results/pub3/reach45_stroke_synerg_',num2str(n),'.mat'];
                filename90 = ['./results/pub3/reach90_stroke_synerg_',num2str(n),'.mat'];
            else
                filename45 = ['./results/pub3/reach45_ctrl.mat'];
                filename90 = ['./results/pub3/reach90_ctrl.mat'];
            end
            u = U(:,:,2);
            x = X(:,:,2);
            y = Y(:,:,2);
            save(filename45,'u','x','y');
            u = U(:,:,3);
            x = X(:,:,3);
            y = Y(:,:,3);
            save(filename90,'u','x','y');
        end
        
    end
    
end
