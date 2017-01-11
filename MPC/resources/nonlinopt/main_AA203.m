tic; clc; clear; close all;
set(0,'DefaultFigureWindowStyle','docked');
addpath( genpath([ pwd '/tbxmanager' ]))
% addpath( genpath('/Users/colesimpson/Documents/Git/motorControl/MPC'))

%% Define initial positions and desired final states
% Initial state:
state.init = [   50;      % Angle of the shoulder
                 30;      % Angle of the elbow
                 10;      % Rate of rotation of the shoulder 
                 0 ];    % Rate of rotation of the elbow
% Desired final state
% state.des = [ 45; 20; 0; 0 ];  % same convention as above
state.des = [ 0.3; 0.5  ];  % location in cartesian space

% Change states to radians
state.init = state.init * pi/180;
% state.des = state.des * pi/180;

%% Simulation parameters
sim.Ts = 2e-2;      % Sampling time (s)
sim.T  = 0.6;         % Simulation time (s)
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

time = 0:sim.Ts:sim.T;
state.current = state.init;
state.history = state.current;
state.controlhistory = [];
state.control = [ 0, 0 ]';
for i = time
    %%% Create MPT model
    % Linearize nonlinear model about current state
    [ A, B, f ] = linearize_xdot(@(x,u) EOM(x,u,params), state.current, ...
        state.control, sim.Ts );
    [ C, D, g ] = linearize_y(@(x,u) toCartesian(x,params), state.current,...
        state.control );
    C = [C(1:2,1:2), zeros(2,2)];     % We don't care about tracking velocity 
    D = [D(1:2,1:2)];     % Cartesian position in the plane.
    g = [g(1:2)];
%     model = LTISystem( 'A', A, 'B', B, 'f', f, 'Ts', sim.Ts );
    model = LTISystem( 'A', A, 'B', B, 'f', f, 'C', C, 'D', D, ...
        'g', g, 'Ts', sim.Ts );

    
    % State constraints
    model.x.min = [ params.th1Min, params.th2Min, -inf, -inf ]' * pi/180;
    model.x.max = [ params.th1Max, params.th2Max,  inf,  inf ]' * pi/180;
    % Controller constraints
    model.u.min = [ params.torq1Min, params.torq2Min ];
    model.u.max = [ params.torq1Max, params.torq2Max ];
    % Make model track a reference (Can be a time-varying reference)
    model.y.with('reference');
    model.y.reference = 'free';
    % Cost function parameter weightings
%     model.x.penalty = QuadFunction( 1e6*diag([ 1, 1, 1e-2, 1e-2 ]));
    model.y.penalty = QuadFunction( 1e6*diag([ 1, 1 ]));
    model.u.penalty = QuadFunction( eye(2) );
    
    % Create MPC controller
    ctrl = MPCController(model, sim.N);

    % Extract the sloptimal control value
    state.control = ctrl.evaluate(state.current, 'y.reference', state.des);

    % Integrate system using optimal torque at this time point
    [t,y] = ode45(@(t,x) EOM(x,state.control,params),...
        [0 sim.Ts], state.current);
    state.current = y(end,:)';
    
    % Store states and commands
    state.history = [ state.history state.current ];
    state.controlhistory = [ state.controlhistory state.control ];
    
    display([ 'Simulation  ' num2str(i./time(end)*100) '% complete (' ...
        num2str(i./sim.T) ' seconds)'])
    display( state.current' )
    toc; tic;
end

time = [ time time(end)+sim.Ts ];

%% Plot results
subplot( 3, 1, 1 )
    plot( time, state.history(1:2,:)' .* 180/pi )
    hold on
    plot( time([1 end]), [ state.des(1:2) state.des(1:2) ]' .* 180/pi, ...
        ':k')
    legend( 'State 1', 'State 2' )
    ylabel 'Joint Angles (deg)'
subplot(3, 1, 2)
    plot( time, state.history(3:4,:)' .* 180/pi )
    ylabel 'Joint Velocities (deg)'
subplot(3,1,3)
    plot( time(2:end), state.controlhistory' )
xlabel 'Time (s)'
ylabel 'Control Effort'

figure('name', 'State Trajectory' )
    plot( state.history(1,:)' * 180/pi, state.history(2,:)' * 180/pi )
    hold on
    plot( state.init(1)*180/pi, state.init(2)*180/pi, 'g*' )
    plot( state.des(1)*180/pi, state.des(2)*180/pi, 'r*' )
    xlabel 'State 1'
    ylabel 'State 2'
    

for i = 1:length( state.controlhistory(1,:))
   state.cartesian(:,i) = toCartesian( state.history(:,i), params); 
end
figure('name', 'Task Space' )
    plot( state.cartesian(1,:), state.cartesian(2,:))
    hold on
    plot( state.des(1), state.des(2), 'r*')
    plot( state.cartesian(1,1), state.cartesian(2,1), 'g*')
    axis('square')
    xlabel 'x-position (m)'
    ylabel 'y-position (m)'
    title 'End-Effector Location'
toc