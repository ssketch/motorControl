% This script plots torques for the weakness reaches as a sanity check.

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
T = 1.5;           % total time to simulate, for each reach [sec]
dt = 0.02;         % time step [sec]
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


%% plot torques from weakness reaches

th = 45;
strength = [0.8;0.4;0.1];
c_sensitiv_strength = autumn(length(strength));

% modify time vector to avoid excessive(ly noisy) plotting
t_f = 0.25;
n_f = length(0:dt:t_f);

% extract/compute & plot control data
data = load(['reach',num2str(th),'_ctrl_slow.mat']);
u_ctrl = data.u(:,1:n_f);

figure()
c_ctrl_u = cmap(c_ctrl,size(u_ctrl,1)+c_buff);
for j = 1:size(u_ctrl,1)
    plot(t(1:n_f)*s2ms,u_ctrl(j,:),...
        'Color',c_ctrl_u(j,:),'LineWidth',thin,'LineSmoothing','on');
    hold on
end

% loop over arm weakness
for s = 1:length(strength)
    
    % average stroke data across trials
    u_stroke = zeros(size(u_ctrl));
    for n = 1:nTrials
        data = load(['reach',num2str(th),'_stroke_weak_c',...
            num2str(strength(s)),'_',num2str(n),'.mat']);
        u_stroke = u_stroke + data.u(:,1:n_f);
    end
    u_stroke = u_stroke/nTrials;
    
    % plot averaged stroke data
    c_weak_u = cmap(c_weak,size(u_stroke,1)+c_buff);
    for j = 1:size(u_stroke,1)
        plot(t(1:n_f)*s2ms,u_stroke(j,:),...
            'Color',c_weak_u(j,:),'LineWidth',thin,'LineSmoothing','on');
    end
    
end

% annotate & save plot
xlim([0 t_f*s2ms])
xlabel('t (ms)','FontSize',fontSize);
ylabel('\tau (Nm)','FontSize',fontSize);
export_fig 'weak_reach45_torques' -transparent -eps
