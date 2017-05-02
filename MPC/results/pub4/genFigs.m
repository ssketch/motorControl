% This script generates the tone/spasticity figures for the NCM 2017
% poster. They will require post-processing in Illustrator.

close all
clear
clc

addpath(genpath([pwd '/../../include']));

%% define global parameters

% movement parameters
nTrials = 1;       % number of trials per condition
p_i = [-0.15;0.3]; % initial position [m]
r = 0.35;          % reach distance [m]
th = 120;          % reach angle [deg]
T = 1;             % total time to simulate, for each reach [sec]
dt = 0.01;         % time step [sec]
t = 0:dt:T;        % time vector [sec]

% plotting parameters
orgShift = -p_i;   % origin shift [m]
m2mm = 1000;       % conversion factor from m to mm
s2ms = 1000;       % conversion factor from sec to msec
thin = 2;
reg = 4;
thick = 6;
small = 18;
large = 25;
fontSize = 16;

%% plot reach data for each MAS score

MAS = [0;1;1.5;2;3;4];
c_spastic = autumn(length(MAS));

% modify time vector to avoid excessive(ly noisy) plotting
t_f = 1;
n_f = length(0:dt:t_f);

% extract/compute & plot stroke data by MAS score, trial-wise
figure()
hold on
for n = 1:length(MAS)
    for i = 1:nTrials
        data = load(['reach',num2str(th),...
            '_stroke_spastic_MAS',num2str(MAS(n)),'_',num2str(i),'.mat']);
        y = data.y;
        p_stroke = (y(1:2,1:n_f) + repmat(orgShift,1,n_f))*m2mm;
        plot(p_stroke(1,:),p_stroke(2,:),...
            'Color',c_spastic(n,:),'LineWidth',thin,'LineSmoothing','on');
    end
end

% compute & plot desired end position
p_f = p_i + r*[cosd(th);sind(th)];
targ = (p_f + orgShift)*m2mm;
plot(targ(1),targ(2),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',large);

% send all targets to bottom layer of plot
targs = findobj(gca, 'MarkerFaceColor', 'k');
uistack(targs, 'bottom');

% annotate & save plot
axis equal
axis([-220 10 -10 345])
xlabel('x (mm)','FontSize',fontSize);
ylabel('y (mm)','FontSize',fontSize);
legend('MAS = 0','MAS = 1','MAS = 1+','MAS = 2','MAS = 3','MAS = 4')

export_fig 'spastic_reach120' -transparent -eps

%% replot reach data for MAS = 4 with reflex forces overlaid

MAS = 4;

% modify time vector to avoid excessive(ly noisy) plotting
t_f = 1;
n_f = length(0:dt:t_f);

% extract/compute & plot stroke data by MAS score, trial-wise
figure()
hold on
for i = 1:nTrials
    data = load(['reach',num2str(th),...
        '_stroke_spastic_MAS',num2str(MAS),'_',num2str(i),'.mat']);
    y = data.y;
    p_stroke = (y(1:2,1:n_f) + repmat(orgShift,1,n_f))*m2mm;
    
    reflex = data.reflex;
    nJoint = size(reflex,1);
    J = data.J(1:nJoint,:,:);
    F_reflex = zeros(2,n_f);
    for j = 1:n_f
        F_reflex(:,j) = (J(:,:,j)')\reflex(:,j);
    end
    
    plot(p_stroke(1,:),p_stroke(2,:),...
        'Color','r','LineWidth',thin,'LineSmoothing','on');
    quiver(p_stroke(1,:),p_stroke(2,:),F_reflex(1,:),F_reflex(2,:),...
        'Color','b');
end

% compute & plot desired end position
p_f = p_i + r*[cosd(th);sind(th)];
targ = (p_f + orgShift)*m2mm;
plot(targ(1),targ(2),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',large);

% annotate & save plot
axis equal
axis([-220 10 -10 345])
xlabel('x (mm)','FontSize',fontSize);
ylabel('y (mm)','FontSize',fontSize);
legend('reach','reflexes')

export_fig 'spastic_MAS4_withReflex' -transparent -eps
