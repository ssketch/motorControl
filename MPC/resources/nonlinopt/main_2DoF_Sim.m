%%% Check the linearization methods

tic; clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');
% addpath( genpath([ pwd '/tbxmanager' ]))
addpath( genpath('/Users/colesimpson/Documents/Git/motorControl/MPC'))

%% Define initial positions and desired final states
% Initial state:
state.init = [   10;      % Angle of the shoulder
                 45;      % Angle of the elbow
                 0;      % Rate of rotation of the shoulder 
                 0 ];    % Rate of rotation of the elbow
% Desired final position
state.des = [ 0.55; 0.5 ];    % Target (in cartesian)
state.des = [ 45; 0; 0; 0 ];  % Target (state space)

% Change states to radians
state.init = state.init * pi/180;


%% Simulation parameters
sim.T  = 1;         % Simulation time (s)

%% Load model parameters
M = 70;          % subject mass [kg]
H = 1.7;         % subject height [m]
params = ModelParameters( M, H );   % Load model parameters scaled to 
                                    % subject's height and weight
clear M H       

%% Simulate using nonlinear function
steps = 10;     % Size grid: torque curve is defined by this many points
Tinit = 1*ones([2,steps]) ;

% Torque limit (lower bound)
lb1 = params.torq1Min * ones( 1, steps );
lb2 = params.torq2Min * ones( 1, steps );
lb = [ lb1; lb2 ];

% Torque limit (upper bound)
ub1 = params.torq1Max * ones( 1, steps );
ub2 = params.torq2Max * ones( 1, steps );
ub = [ ub1; ub2 ];

% Optimize!
% T = fmincon(@(T) ReachCost_2dof_Cart(T, state, sim, params), Tinit, ...
%     [], [], [], [], lb, ub );
T = fmincon(@(T) ReachCost_2dof(T, state, sim, params), Tinit, ...
    [], [], [], [], lb, ub );

% Simulate the reach using the optimal torques
time = linspace( 0, sim.T, length( T(1,:)) );
[t,y] = ode45(@(t,x) dxdt(t,x,T,time,params),[0 sim.T], state.init);
T_interp = [ interp1(time,T(1,:),t,'spline')'; ...
    interp1(time,T(2,:),t,'spline')' ];

%% Figure 1: Plots vs. Time
subplot( 3,1,1 )
    plot( t, y(:,1) * 180/pi )
    hold on
    plot( t, y(:,2) * 180/pi )
    ylabel '\theta (deg)'
    box off
subplot( 3,1,2 )
    plot( t, y(:,3) * 180/pi )
    hold on
    plot( t, y(:,4) * 180/pi )
    h1 = ylabel( '$\dot{\theta}$ (deg/sec)' );
    set( h1, 'Interpreter', 'latex' );
    box off
subplot( 3,1,3)
    plot( time, T, '*')
    hold on
    plot( t, T_interp )
    ylabel '\tau (N-m)'
    xlabel 'Time (sec)'
    box off
    
%% Figure 2: Task Space
for i = 1:length( t)
   state.history(:,i) = toCartesian( y(i,:), params); 
end
figure('name', 'Task Space' )
    plot( state.history(1,:), state.history(2,:))
    hold on
    plot( state.des(1), state.des(2), 'r*')
    plot( state.history(1,1), state.history(2,1), 'g*')
    axis('square')
    xlabel 'x-position (m)'
    ylabel 'y-position (m)'
    title 'End-Effector Location'

%% Figure 3: Joint Space
figure('name', 'Joint Space' )
    plot( y(:,1)*180/pi, y(:,2)*180/pi)
    hold on
    plot( y(1,1)*180/pi, y(1,2)*180/pi, '*g')
    plot( y(end,1)*180/pi, y(end,2)*180/pi, 'r*' )
    box off
    