% This script generates the figures for ICORR 2017. They will require
% post-processing in Illustrator.

close all
clear
clc

addpath(genpath([pwd '/../../include']));

%% define global parameters

% movement parameters
nDeficit = 3;      % number of deficits being tested (synergy, sensing, weakness)
nTrials = 5;       % number of trials per condition
p_i = [-0.15;0.3]; % initial position [m]
r = 0.15;          % reach distance [m]
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
colors = linspecer(nDeficit+1); % +1 for control
c_ctrl = colors(1,:);
c_syng = colors(2,:);
c_sens = colors(3,:);
c_weak = colors(4,:);
c_buff = 2; % buffer so colors don't get too light with "cmap" function

%% plot synergy data for center-out reaches

nReach = 8;               % total number of (evenly spaced) center-out reaches
thStep = 360/nReach;      % step from one reach angle to next [deg]
th = 0:thStep:360-thStep; % reach angles [deg]

% modify time vector to avoid excessive(ly noisy) plotting
t_f = 0.8;
n_f = length(0:dt:t_f);

% load control data
circle = load('circle_ctrl.mat');

% average data
Yavg = zeros(size(circle.Y));
for n = 1:nTrials
    data = load(['circle_stroke_synerg_',num2str(n),'.mat']);
    Yavg = Yavg + data.Y;
end
Yavg = Yavg/nTrials;

% plot averaged data
figure()
hold on
for i = 1:length(th)
    
    % compute desired end position
    p_f = p_i + r*[cosd(th(i));sind(th(i))];
    
    % shift and scale data for plotting
    targ = (p_f + orgShift)*m2mm;
    p_ctrl = (circle.Y(1:2,1:n_f,i) + repmat(orgShift,1,n_f))*m2mm;
    p_stroke = (Yavg(1:2,1:n_f,i) + repmat(orgShift,1,n_f))*m2mm;
    
    % plot target and reaches
    plot(targ(1),targ(2),'o',...
        'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',large);
    plot(p_ctrl(1,:),p_ctrl(2,:),...
        'Color',c_ctrl,'LineWidth',reg,'LineSmoothing','on');
    plot(p_stroke(1,:),p_stroke(2,:),...
        'Color',c_syng,'LineWidth',thick,'LineSmoothing','on');
    
end

% send all targets to bottom layer of plot
targs = findobj(gca, 'MarkerFaceColor', 'k');
uistack(targs, 'bottom');

% bring all stroke reaches to top layer of plot
stroke_reaches = findobj(gca, 'Color', c_syng);
uistack(stroke_reaches, 'top');

% annotate & save plot
axis equal
box on
axis([-180 180 -180 180])
xlabel('x (mm)','FontSize',fontSize);
ylabel('y (mm)','FontSize',fontSize);
export_fig 'synerg_centerOut' -transparent -eps

%% plot 45 & 90 degree synergy reach data

th = [45;90];

% modify time vector to avoid excessive(ly noisy) plotting
t_f = 0.8;
n_f = length(0:dt:t_f);

% loop over angles
for i = 1:length(th)
    
    figure()
    
    % compute desired end position
    p_f = p_i + r*[cosd(th(i));sind(th(i))];
    targ = (p_f + orgShift)*m2mm;
    
    % extract/compute control data
    data = load(['reach',num2str(th(i)),'_ctrl.mat']);
    y = data.y;
    p_ctrl = (y(1:2,1:n_f) + repmat(orgShift,1,n_f))*m2mm;
    vX = y(4,1:n_f)*m2mm;
    vY = y(5,1:n_f)*m2mm;
    vT_ctrl = vX*cosd(th(i)) + vY*sind(th(i));
    
    % plot control data (both panels)
    subplot(1,2,1)
    plot(targ(1),targ(2),'o',...
        'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',large);
    hold on
    plot(p_ctrl(1,:),p_ctrl(2,:),...
        'Color',c_ctrl,'LineWidth',thin,'LineSmoothing','on');
    subplot(1,2,2)
    plot(t(1:n_f)*s2ms,vT_ctrl,...
        'Color',c_ctrl,'LineWidth',thin,'LineSmoothing','on');
    hold on
    
    % extract/compute & plot stroke data, trial-wise
    for n = 1:nTrials
        data = load(['reach',num2str(th(i)),'_stroke_synerg_',num2str(n),'.mat']);
        y = data.y;
        p_stroke = (y(1:2,1:n_f) + repmat(orgShift,1,n_f))*m2mm;
        vX = data.y(4,1:n_f)*m2mm;
        vY = data.y(5,1:n_f)*m2mm;
        vT_stroke = vX*cosd(th(i)) + vY*sind(th(i));
        
        subplot(1,2,1)
        plot(p_stroke(1,:),p_stroke(2,:),...
            'Color',c_syng,'LineWidth',thin,'LineSmoothing','on');
        subplot(1,2,2)
        plot(t(1:n_f)*s2ms,vT_stroke,...
            'Color',c_syng,'LineWidth',thin,'LineSmoothing','on');
    end
    
    % annotate & save plots
    if (th(i) == 45)
        subplot(1,2,1)
        axis equal
        axis([-10 175 -30 200])
        xlabel('x (mm)','FontSize',fontSize);
        ylabel('y (mm)','FontSize',fontSize);
        
        subplot(1,2,2)
        xlim([0 t_f*s2ms])
        xlabel('t (ms)','FontSize',fontSize);
        ylabel('v_t (^m^m/_s)','FontSize',fontSize);
        
        export_fig 'synerg_reach45' -transparent -eps
    else
        subplot(1,2,1)
        axis equal
        axis([-50 30 0 180])
        xlabel('x (mm)','FontSize',fontSize);
        ylabel('y (mm)','FontSize',fontSize);
        
        subplot(1,2,2)
        xlim([0 t_f*s2ms])
        xlabel('t (ms)','FontSize',fontSize);
        ylabel('v_t (^m^m/_s)','FontSize',fontSize);
        
        export_fig 'synerg_reach90' -transparent -eps
    end
    
end

%% create sensitivity plots for sensing error

err = [8;2;0.2];
c_sensitiv_err = autumn(length(err));

% modify time vector to avoid excessive(ly noisy) plotting
t_f = 0.8;
n_f = length(0:dt:t_f);

% compute desired end position
th = 45;
p_f = p_i + r*[cosd(th);sind(th)];
targ = (p_f + orgShift)*m2mm;

% extract/compute & plot control data
data = load(['reach',num2str(th),'_ctrl.mat']);
x = data.x(:,1:n_f);
y = data.y(:,1:n_f);
p_ctrl = (y(1:2,:) + repmat(orgShift,1,n_f))*m2mm;
th_ctrl = x(1:2,:);

figure()
subplot(1,2,1)
plot(targ(1),targ(2),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',large);
hold on
plot(p_ctrl(1,:),p_ctrl(2,:),...
    'Color',c_ctrl,'LineWidth',thin,'LineSmoothing','on');
subplot(1,2,2)
for j = 1:size(th_ctrl,1)
    plot(t(1:n_f)*s2ms,th_ctrl(j,:),...
        'Color',c_ctrl,'LineWidth',thin,'LineSmoothing','on');
    hold on
end

% loop over severity of prediction error
for e = 1:length(err)
    
    % average stroke data across trials
    p_stroke = zeros(size(p_ctrl));
    th_stroke = zeros(size(th_ctrl));
    for n = 1:nTrials
        data = load(['reach',num2str(th),'_stroke_predErr_n',...
            num2str(err(e)),'_',num2str(n),'.mat']);
        x = data.x(:,1:n_f);
        y = data.y(:,1:n_f);
        p_stroke = p_stroke + (y(1:2,:) + repmat(orgShift,1,n_f))*m2mm;
        th_stroke = th_stroke + x(1:2,:);
    end
    p_stroke = p_stroke/nTrials;
    th_stroke = th_stroke/nTrials;
    
    % plot averaged stroke data
    subplot(1,2,1)
    c_sens = c_sensitiv_err(e,:);
    plot(p_stroke(1,:),p_stroke(2,:),...
        'Color',c_sens,'LineWidth',thin,'LineSmoothing','on');
    subplot(1,2,2)
    for j = 1:size(th_stroke,1)
        plot(t(1:n_f)*s2ms,th_stroke(j,:),...
            'Color',c_sens,'LineWidth',thin,'LineSmoothing','on');
    end
    
end

% annotate & save plot
subplot(1,2,1)
axis equal
box on
axis([-10 160 -10 140])
xlabel('x (mm)','FontSize',fontSize);
ylabel('y (mm)','FontSize',fontSize);
subplot(1,2,2)
xlim([0 t_f*s2ms])
xlabel('t (ms)','FontSize',fontSize);
ylabel('\theta (deg)','FontSize',fontSize);
export_fig 'sens_reach45_sensitivity' -transparent -eps

%% create sensitivity plots for muscular weakness

strength = [0.1;0.4;0.8];
c_sensitiv_strength = autumn(length(strength));

% modify time vector to avoid excessive(ly noisy) plotting
t_f = 0.8;
n_f = length(0:dt:t_f);

% compute desired end position
th = 45;
p_f = p_i + r*[cosd(th);sind(th)];
targ = (p_f + orgShift)*m2mm;

% extract/compute & plot control data
data = load(['reach',num2str(th),'_ctrl.mat']);
x = data.x(:,1:n_f);
y = data.y(:,1:n_f);
p_ctrl = (y(1:2,:) + repmat(orgShift,1,n_f))*m2mm;
th_ctrl = x(1:2,:);

figure()
subplot(1,2,1)
plot(targ(1),targ(2),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','k','MarkerSize',large);
hold on
plot(p_ctrl(1,:),p_ctrl(2,:),...
    'Color',c_ctrl,'LineWidth',thin,'LineSmoothing','on');
subplot(1,2,2)
for j = 1:size(th_ctrl,1)
    plot(t(1:n_f)*s2ms,th_ctrl(j,:),...
        'Color',c_ctrl,'LineWidth',thin,'LineSmoothing','on');
    hold on
end

% loop over arm weakness
for s = 1:length(strength)
    
    % average stroke data across trials
    p_stroke = zeros(size(p_ctrl));
    th_stroke = zeros(size(th_ctrl));
    for n = 1:nTrials
        data = load(['reach',num2str(th),'_stroke_weak_c',...
            num2str(strength(s)),'_',num2str(n),'.mat']);
        x = data.x(:,1:n_f);
        y = data.y(:,1:n_f);
        p_stroke = p_stroke + (y(1:2,:) + repmat(orgShift,1,n_f))*m2mm;
        th_stroke = th_stroke + x(1:2,:);
    end
    p_stroke = p_stroke/nTrials;
    th_stroke = th_stroke/nTrials;
    
    % plot averaged stroke data
    subplot(1,2,1)
    c_weak = c_sensitiv_strength(s,:);
    plot(p_stroke(1,:),p_stroke(2,:),...
        'Color',c_weak,'LineWidth',thin,'LineSmoothing','on');
    subplot(1,2,2)
    for j = 1:size(th_stroke,1)
        plot(t(1:n_f)*s2ms,th_stroke(j,:),...
            'Color',c_weak,'LineWidth',thin,'LineSmoothing','on');
    end
    
end

% annotate & save plot
subplot(1,2,1)
axis equal
box on
axis([-10 160 -10 140])
xlabel('x (mm)','FontSize',fontSize);
ylabel('y (mm)','FontSize',fontSize);
subplot(1,2,2)
xlim([0 t_f*s2ms])
xlabel('t (ms)','FontSize',fontSize);
ylabel('\theta (deg)','FontSize',fontSize);
export_fig 'weak_reach45_sensitivity' -transparent -eps
