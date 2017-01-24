close all
clear
clc

% define constants
nTrials = 3;
deficits = {'synerg_';'estErr_';'synergANDestErr_'};

% define reach parameters
origin1 = [-0.15;0.3;0];  % origin 1 (arbitrary) [m]
origin2 = [-0.18;0.56;0]; % origin 2, to match (Beer, 2000) [m]
p_i = origin1;            % initial position [m]
r = 0.15;                 % reach distance [m]

% define plotting parameters, to match (Beer, 2000) figure
orgShift = -p_i; % [m]
m2mm = 1000;
xMin = -220; % [mm]
xMax =  220; % [mm]
yMin = -220; % [mm]
yMax =  220; % [mm]
GREEN = [68 170 76]*(1/255);
RED = [214 42 49]*(1/255);
GRAY = [78 78 77]*(1/255);
lineThickness = 4;
markerSize = 21;
fontSize = 14;

% load control data
ctrl = load('../../results/origin1/circle_ctrl.mat');

% loop over deficits
for d = 1:length(deficits)
    
    % create new figure
    figure()
    hold on
    
    % extract control data and associated parameters
    Y_ctrl = ctrl.Y;
    nReach = size(Y_ctrl,3);
    thStep = 360/nReach;
    
    % load & average stroke data
    Yavg_stroke = zeros(size(ctrl.Y));
    for n = 1:nTrials
        data = load(['../../results/origin1/circle_stroke_',deficits{d},num2str(n),'.mat']);
        Yavg_stroke = Yavg_stroke + data.Y;
    end
    Yavg_stroke = Yavg_stroke/nTrials;
    
    % plot control vs. stroke data
    for i = 1:nReach
        
        % compute desired end position
        th = thStep*(i-1);
        p_f = p_i + r*[cosd(th);sind(th);0];
        
        % shift and scale data for plotting
        targ = (p_f + orgShift)*m2mm;
        pAct_ctrl = Y_ctrl(1:3,:,i);
        pAct_stroke = Yavg_stroke(1:3,:,i);
        pShift_ctrl = pAct_ctrl + repmat(orgShift,1,size(pAct_ctrl,2));
        pShift_stroke = pAct_stroke + repmat(orgShift,1,size(pAct_stroke,2));
        p_ctrl = pShift_ctrl*m2mm;
        p_stroke = pShift_stroke*m2mm;

        % plot target and reaches
        plot3(targ(1),targ(2),targ(3),'o',...
            'MarkerEdgeColor',GRAY,'MarkerFaceColor',GRAY,'MarkerSize',markerSize);
        plot3(p_ctrl(1,:),p_ctrl(2,:),p_ctrl(3,:),...
            'Color',GREEN,'LineWidth',lineThickness,'LineSmoothing','on');
        plot3(p_stroke(1,:),p_stroke(2,:),p_stroke(3,:),...
            'Color',RED,'LineWidth',lineThickness,'LineSmoothing','on');
        
    end
    
    % send all targets to bottom layer of plot
    targs = findobj(gca, 'MarkerFaceColor', GRAY);
    uistack(targs, 'bottom');
    
    % bring all stroke reaches to top layer of plot
    stroke_reaches = findobj(gca, 'Color', RED);
    uistack(stroke_reaches, 'top');
    
    % annotate and save plot
    view(0,90)
    axis equal
    box on
    xlim([xMin xMax])
    ylim([yMin yMax])
    xlabel('x (mm)','FontSize',fontSize);
    ylabel('y (mm)','FontSize',fontSize);
    zlabel('z (mm)','FontSize',fontSize);
    export_fig '../../results/origin1/posTrajs_circle_avg' -transparent -pdf -append
    
end
