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
synerg = 1;          % 1 = stroke coupled muscle synergies
weak = 0;            % 1 = stroke caused muscular weakness

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

% define movement parameters
nReach = 8;                  % total number of (evenly spaced) center-out reaches
nTrials = 5;                 % number of times to repeat 'nReach' trials
T = 0.75;                    % total time to simulate, for each reach [sec]
movt.t = 0:arm.Ts:T;         % time vector [sec]
r = 0.15;                    % reach distance [m]
thStep = 360/nReach;         % step from one reach angle to next [deg]
th = 0:thStep:360-thStep;    % reach angles [deg]
origin1 = [-0.15;0.3;0];     % origin 1 (arbitrary) [m]
origin2 = [-0.18;0.56;0];    % origin 2, to match (Beer, 2000) [m]
origin3 = [-0.15;0.6;0];     % origin 3,(less arbitrary) [m]
p_i = origin1;               % initial position [m]
v_i = [0;0;0];               % initial velocity [m/s]
y_i = [p_i;v_i];             % initial state, in Cartesian coordinates [m,m/s]
[x_i,~,~] = arm.invKin(y_i); % initial state, in joint coordinates [rad,rad/s]
movt.space = 'task';         % space in which to track reference ('joint' or 'task')

% define plotting parameters, to match (Beer, 2000) figure
plotOn = 1;
orgShift = -p_i;
m2mm = 1000;
s2ms = 1000;
xMin = -220; % [mm]
xMax =  220; % [mm]
yMin = -220; % [mm]
yMax =  220; % [mm]
colors = linspecer(2);
col_ctrl = colors(1,:);
col_stroke = colors(2,:);
lineThickness = 4;
markerSize = 21;
fontSize = 14;

for n = 1%1:nTrials
    
    if plotOn
        figure()
        hold on
    end
    
    for stroke = 1%0:1
        
        % reseed random number generator for consistency between stroke and
        % control reaches
        rng(10*n)
        
        % if stroke, update model parameters
        if stroke
            if predErr
                % from (Yousif, 2015), for deafferented patient
                toRad = pi/180;
                posNoise = 10;
                arm.sensNoise(1:nJoints) = posNoise*ones(nJoints,1)*toRad;
                biasData_stroke(:,:,1) = [25 -8;35 -2;50 6]*toRad;
                biasData_stroke(:,:,2) = [80 -8;90 0;100 6]*toRad;
                arm.sensBias = defineBiasFunc(biasData_stroke);
                
                % prediction noise
                intModel.motrNoise = 1.8;
            end
            if synerg
                % from (Dewald, 1995), representing muscle synergies induced by stroke
                Msynerg = [1, 0, 0, 0; 0, 1, 0.13, 0.63;
                           0, 0, 1, 0; 0.8, 0.06, 0, 1]; % representing muscle synergies (Dewald, 1995)
                for i = 1:size(Msynerg,1)
                    Msynerg(i,:) = Msynerg(i,:) / sum(Msynerg(i,:)); % normalization
                end
                arm.coupling = arm.coupling * Msynerg;
            end
            if weak
                % arbitrary scalar for control limits in MPC
                % (NOTE: imposed on internal model because it is used for control)
                intModel.strength = 0.3;
            end
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
            % NOTE: internal model's state estimates are grounded by vision (i.e.,
            % ----  assuming perfect vision, they match the arm's actual state)
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
            
            if plotOn
                
                % shift and scale position data for plotting
                pAct = data.yAct(1:3,:);
                pShift = pAct + repmat(orgShift,1,size(pAct,2));
                p = pShift*m2mm;
                targ = (p_f + orgShift)*m2mm;
                
                % plot trajectory (and target, if necessary)
                if stroke
                    plot3(p(1,:),p(2,:),p(3,:),...
                        'Color',col_stroke,'LineWidth',lineThickness,'LineSmoothing','on');   % stroke
                else
                    plot3(p(1,:),p(2,:),p(3,:),...
                        'Color',col_ctrl,'LineWidth',lineThickness,'LineSmoothing','on');     % control
                    plot3(targ(1),targ(2),targ(3),'o',...
                        'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',markerSize); % target
                end
                view(0,90)
                axis equal
                box on
                xlim([xMin xMax])
                ylim([yMin yMax])
                
            end
            
            % save state trajectories
            U(:,:,i) = data.uCmd;
            X(:,:,i) = data.xAct;
            Y(:,:,i) = data.yAct;
            
        end
        
        % save all trajectories into MAT file
        if stroke
            filename = ['./results/pub2/circle_stroke_synerg_',num2str(n),'.mat'];
        else
            filename = ['./results/pub2/circle_ctrl_',num2str(n),'.mat'];
        end
        save(filename,'U','X','Y');
        
        % if 8 reaches, save specific trajectories into MAT files
        if nReach == 8
            if stroke
                filename45 = ['./results/pub2/reach45_stroke_synerg_',num2str(n),'.mat'];
                filename90 = ['./results/pub2/reach90_stroke_synerg_',num2str(n),'.mat'];
            else
                filename45 = ['./results/pub2/reach45_ctrl',num2str(n),'.mat'];
                filename90 = ['./results/pub2/reach90_ctrl',num2str(n),'.mat'];
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

if plotOn
    % send all targets to bottom layer of plot
    targs = findobj(gca, 'MarkerFaceColor', 'k');
    uistack(targs, 'bottom');
    
    % bring all stroke reaches to top layer of plot
    stroke_reaches = findobj(gca, 'Color', col_stroke);
    uistack(stroke_reaches, 'top');
    
    % annotate and save plot
    view(0,90)
    axis equal
    grid on
    box on
    xlim([xMin xMax])
    ylim([yMin yMax])
    xlabel('x (mm)','FontSize',fontSize);
    ylabel('y (mm)','FontSize',fontSize);
    zlabel('z (mm)','FontSize',fontSize);
end
