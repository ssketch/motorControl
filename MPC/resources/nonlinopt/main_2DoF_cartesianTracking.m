tic; clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');
addpath( genpath([ pwd '/tbxmanager' ]))


%% Define initial positions and desired final states
% Initial state:
state.init = [   50;      % Angle of the shoulder
                 30;      % Angle of the elbow
                 0;      % Rate of rotation of the shoulder 
                 0 ];    % Rate of rotation of the elbow
% Desired final state
state.des = [ 45; 20; 0; 0 ];  % same convention as above

% Change states to radians
state.init = state.init * pi/180;
state.des.jointspace = state.des * pi/180;

%% Simulation parameters
sim.Ts = 1e-1;      % Sampling time (s)
sim.T  = 6;         % Simulation time (s)
sim.N  = 15;        % Window, number of time points over which to optimize
                    % on each iteration

%% Load model parameters
M = 70;          % subject mass [kg]
H = 1.7;         % subject height [m]
params = ModelParameters( M, H );   % Load model parameters scaled to 
                                    % subject's height and weight
clear M H       

%% Create MPC controller using MPT3
A = zeros( length( state.init), length( state.init ));
B = [ zeros( 2, 2); eye(2)./params.tau ];

state.des.cartesian = toCartesian( state.des.jointspace, params );


time = 0:sim.Ts:sim.T;
state.current = state.init;
state.history.jointspace = state.current;
state.history.cartesian = toCartesian( state.current, params );
state.controlhistory = [];
state.control = [ 0, 0 ]';
for i = time
    
    % Create MPC model
    % Linearize nonlinear model about current state
    A = linearize( @(x) forward_2dof(x, state.control, params, sim), ...
        state.current );
    C = linearize( @(x) toCartesian( x, params ), state.current );
    model = LTISystem( 'A', A, 'B', B, 'C', C, 'Ts', sim.Ts );
    % model = nonlin2PWA( params );

    % State constraints
    model.x.min = [ params.th1Min, params.th2Min, -inf, -inf ]' * pi/180;       
    model.x.max = [ params.th1Max, params.th2Max,  inf,  inf ]' * pi/180;
    % Controller constraints
    model.u.min = [ params.torq1Min, params.torq2Min ];
    model.u.max = [ params.torq1Max, params.torq2Max ];
    % Cost function parameter weightings
    model.y.penalty = QuadFunction( 1 * diag([ 1, 1, 0.1, 0.1 ]));
%     model.x.penalty = QuadFunction( 1 * diag([ 0, 0, 1, 1 ]));
    model.u.penalty = QuadFunction( 1*eye(2) );
    % Make model track a reference (Can be a time-varying reference)
    model.y.with('reference');
    model.y.reference = 'free';

    % Add a terminal cost to the controller
%     PN = model.LQRPenalty;
%     model.x.with('terminalPenalty');
%     model.x.terminalPenalty = PN;

    % Create MPC controller
    ctrl = MPCController(model, sim.N);

    % Simulate the closed-loop system
    loop = ClosedLoop(ctrl, model);
    state.control = ctrl.evaluate(state.current, 'y.reference', ...
        state.des.cartesian );
    
    % Iterate system using optimal torque at this time point
    state.current = forward_2dof( state.current, state.control, ...
        params, sim);
    % Store states and commands
    state.history.jointspace = [ state.history.jointspace, ...
        state.current ];
    state.history.cartesian = [ state.history.cartesian, ...
        toCartesian( state.current, params )];
    state.controlhistory = [ state.controlhistory state.control ];
    
    display([ 'Step ' num2str(i)])
end

time = [ time time(end)+sim.Ts ];

subplot( 3, 1, 1 )
    plot( time, state.history.jointspace(1:2,:)' .* 180/pi )
    hold on
    plot( time([1 end]), [ state.des.jointspace(1:2), ...
        state.des.jointspace(1:2) ]' .* 180/pi, ':k')
    legend( 'State 1', 'State 2' )
    ylabel 'Joint Angles (deg)'
subplot(3, 1, 2)
    plot( time, state.history.jointspace(3:4,:)' .* 180/pi )
    ylabel 'Joint Velocities (deg)'
subplot(3,1,3)
    plot( time(2:end), state.controlhistory' )
xlabel 'Time (s)'
ylabel 'Control Effort'

figure
    plot( state.history.cartesian(1,:), state.history.cartesian(2,:))
    hold on
    start = toCartesian( state.init, params);
    plot( start(1), start(2), 'g*' )
    plot( state.des.cartesian(1), state.des.cartesian(2), 'r*' )
    xlabel 'X (m)'
    ylabel 'Y (m)'

toc