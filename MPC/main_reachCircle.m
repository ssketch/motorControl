close all
clear
clc

% add folders to path
addpath(genpath([pwd '/include']));

% define subject
subj.hand = 'right'; % hand being tested
subj.M = 70;         % mass [kg]
subj.H = 1.80;       % height [meters]
estErr = 0;          % 1 = stroke caused estimation error
synerg = 1;          % 1 = stroke coupled muscle synergies

% define subject's physical arm & internal arm model (mental)
arm = arm_2DOF(subj);
intModel = arm_2DOF(subj);

% extract arm parameters
nInputs = length(arm.u.val);
nJoints = length(arm.q.val);
nStatesTsk = length(arm.y.val);

% define movement parameters
nReach = 16;                        % total number of (evenly spaced) center-out reaches
T = 1;                              % total time to simulate, for each reach [sec]
movt.t = 0:arm.Ts:T;                % time vector [sec]
d = 0.15;                           % reach distance [m]
thStep = 360/nReach;                % step from one reach angle to next [deg]
th = 0:thStep:360-thStep;           % reach angles [deg]
%thStep = 45;                        % step from one reach angle to next [deg]
%th = 0:thStep:180;                  % reach angles [deg]
p_i = [-0.15;0.3;0];                % initial position [m]
v_i = [0;0;0];                      % initial velocity [m/s]
y_i = [p_i;v_i];                    % initial state, in Cartesian coordinates [m,m/s]
[x_i,~,~] = arm.invKin(y_i);        % initial state, in joint coordinates [rad,rad/s]
movt.space = 'task';                % space in which to track reference ('joint' or 'task')

% loop over stroke
for stroke = 1
    
    % if stroke, update model parameters
    if stroke
        if estErr
            toRad = pi/180;
            posNoise = 10; % (Yousif, 2015)
            arm.sensNoise(1:nJoints) = posNoise*ones(nJoints,1)*toRad;
            biasData_stroke(:,:,1) = [25 -8;35 -2;50 6]*toRad; % (Yousif, 2015)
            biasData_stroke(:,:,2) = [80 -8;90 0;100 6]*toRad;
            arm.sensBias = defineBiasFunc(biasData_stroke);
            intModel.motrNoise = 1; % prediction noise (arbitrary, 1 = largest possible (OOM) without crashing the optimization)
        end
        if synerg
            arm.coupling = arm.coupling * [ 1, 0, 0, 0; 0, 0.5682, 0.0739, 0.3580;
                     0, 0, 1, 0; 0.4301, 0.0323, 0, 0.5376 ]; % representing muscle synergies
            intModel.coupling = intModel.coupling * [ 1, 0, 0, 0; 0, 0.5682, 0.0739, 0.3580;
                     0, 0, 1, 0; 0.4301, 0.0323, 0, 0.5376 ]; % representing muscle synergies

            for n = 1:size(arm.coupling,1)
                arm.coupling(n,:) = arm.coupling(n,:) / sum(arm.coupling(n,:)); % normalization
            end
        end
    end
    
    % loop over circle of reaches
    for i = 1:length(th)
        
        % define movement reference trajectory
        p_f = p_i + d*[cosd(th(i));sind(th(i));0]; % desired end position [m]
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
        
        % simulate reach
        data = simulate(movt, arm, intModel);
        
        % save (downsampled) task-space position trajectory for plotting
        f_down = 1;
        pAct(:,:,i) = downsample(data.yAct(1:nStatesTsk/2,:)',f_down)';
        
    end
    
    % save all task-space position trajectories into MAT file
    if stroke
        filename = './results/circle_stroke.mat';
    else
        filename = './results/circle_ctrl.mat';
    end
    save(filename,'pAct');
    
    % define plotting parameters
    green = [68 170 76]*(1/255);
    red = [214 42 49]*(1/255);
    
    % plot trajectories
    for i = 1:length(th)
        p = pAct(:,:,i);
        if stroke
            plot3(p(1,:),p(2,:),p(3,:),'r--','LineWidth',4,'LineSmoothing','on'); % stroke
        else
            plot3(p(1,:),p(2,:),p(3,:),'b','LineWidth',4,'LineSmoothing','on');   % control
        end
        hold on
    end
    
end

% annotate and save plot
axis equal
view(0,90)
xlabel('x','FontSize',28);
ylabel('y','FontSize',28);
zlabel('z','FontSize',28);
export_fig './results/posTraj_circle' -transparent -eps
