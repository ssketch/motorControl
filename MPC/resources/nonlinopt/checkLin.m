%%% Check the linearization methods

tic; clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');
% addpath( genpath([ pwd '/tbxmanager' ]))
addpath( genpath('/Users/colesimpson/Documents/Git/motorControl/MPC'))

%% Define initial positions and desired final states
% Initial state:
state.init = [   5;      % Angle of the shoulder
                 50;      % Angle of the elbow
                 0;      % Rate of rotation of the shoulder 
                 0 ];    % Rate of rotation of the elbow
% Desired final state
state.des = [ 10; 15; 0; 0 ];  % same convention as above

% Change states to radians
state.init = state.init * pi/180;
state.des = state.des * pi/180;

%% Simulation parameters
% sim.Ts = 1e-2;      % Sampling time (s)
sim.T  = 2;         % Simulation time (s)

%% Load model parameters
M = 70;          % subject mass [kg]
H = 1.7;         % subject height [m]
params = ModelParameters( M, H );   % Load model parameters scaled to 
                                    % subject's height and weight
clear M H       

%% Simulate using nonlinear function
steps = 10;
Tinit = 10*zeros([2,steps]) ;

lb1 = params.torq1Min * ones( 1, steps );
lb2 = params.torq2Min * ones( 1, steps );
lb = [ lb1; lb2 ];

ub1 = params.torq1Max * ones( 1, steps );
ub2 = params.torq2Max * ones( 1, steps );
ub = [ ub1; ub2 ];


T = fmincon(@(T) ReachCost_2dof(T, state, sim, params), Tinit, ...
    [], [], [], [], lb, ub );

    
time = linspace( 0, sim.T, length( T(1,:)) );
[t,y] = ode45(@(t,x) dxdt(t,x,T,time,params),[0 sim.T], state.init);
T_interp = [ interp1(time,T(1,:),t,'spline')'; ...
    interp1(time,T(2,:),t,'spline')' ];

subplot( 3,1,1 )
    plot( t, y(:,1) * 180/pi )
    hold on
    plot( t, y(:,2) * 180/pi )
    plot( t, state.des(1)*180/pi*ones(size(t)), 'k:')
    plot( t, state.des(2)*180/pi*ones(size(t)), 'k:')
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
    
