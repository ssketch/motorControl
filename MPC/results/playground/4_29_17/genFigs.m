close all
clear
clc

addpath(genpath([pwd '/../../../include']));

%% define global parameters

% movement parameters
cond = {'ctrl';
        'spastic';
        'synerg';
        'synerg_spastic'}; % conditions tested
nTrials = 1;               % number of trials per condition
p_i = [-0.15;0.3];         % initial position [m]
r = 0.15;                  % reach distance [m]
T = 1;                     % total time to simulate, for each reach [sec]
dt = 0.01;                 % time step [sec]
t = 0:dt:T;                % time vector [sec]

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
colors = linspecer(length(cond));

%% plot center-out reaches

th = [45;90;135];              % reach angles [deg]

% modify time vector to avoid excessive(ly noisy) plotting
t_f = 0.8;
n_f = length(0:dt:t_f);

% plot reaches
figure()
hold on
for i = 1:length(th)
    for n = 1:length(cond)
        
        % extract relevant data
        strToFind = ['reach',num2str(th(i)),'_',cond{n},'*.mat'];
        file = dir(strToFind);
        data = load(file(1).name);
        
        % shift, scale, & plot data
        p = (data.y(1:2,1:n_f) + repmat(orgShift,1,n_f))*m2mm;
        plot(p(1,:),p(2,:),...
            'Color',colors(n,:),'LineWidth',reg,'LineSmoothing','on');
        
    end
end
legend('control','spastic','synergy','spastic + synergy')

% plot and send all targets to bottom layer of plot
for i = 1:length(th)
    p_f = p_i + r*[cosd(th(i));sind(th(i))];
    targ = (p_f + orgShift)*m2mm;
    plot(targ(1),targ(2),'o',...
        'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',large);
end
targs = findobj(gca, 'MarkerFaceColor', 'k');
uistack(targs, 'bottom');

% annotate & save plot
axis equal
box on
axis([-180 180 -10 180])
title('15cm Center-out Reaches','FontSize',fontSize);
xlabel('x (mm)','FontSize',fontSize);
ylabel('y (mm)','FontSize',fontSize);
export_fig 'synerg_spastic_centerOut' -transparent -eps
