clear; clc; close all;
%%% This script computes the workspace for the simulated arm

%% Step 1: Setup arm and MPC toolbox
% Initialize MPT3 toolbox
addpath( genpath([pwd '/tbxmanager']));

% Subject characteristics
subj.M = 70;    % kg
subj.H = 1.80;  % meters
subj.hand = 'right';
subj.Td = 0;
subj.coupled = false;

% Define a model.  We'll start with the 2 degree of freedom planar model
model = arm_2DOF(subj);

% Initially, the model will be resting at 0 degrees of shoulder horizontal
% rotation and 0 degrees of elbow flexion.
model.q = [mean( model.thLim(1:2,:), 2 ); 0; 0 ];
% model.q = [ model.thLim(1:2,1); 0; 0 ];
model.draw;

histories.u = zeros(model.tDOF, 1);
histories.q = model.q;
histories.y = fwdKin( model, model.q );

for th = ( 0:30:360 )*pi/180
    display(['Reach direction: theta = ' num2str(th)*180/pi 'Deg'])
    display('_________________________________')
    
    r = 10;
    ref = [ r*cos(th); r*sin(th); 0; 0 ];
    % Perform a reach:
    i = 0;
    q_diff = diff( histories.q' );
    model.q = [mean( model.thLim(1:2,:), 2 ); 0; 0 ];


    %% Simulate reach.
    % This while loop simulates a movement until the model stops moving (joint
    % velocities and accelerations decrease below 1e-2 radians or
    % radians/second) as long as the resulting state is within the joint
    % limits.  This loop also assumes that the movement will take at least 0.1
    % seconds.
    while model.withinLimits && ( max(abs( q_diff(end,:))) > 1e-2  ...
            || i*model.Ts < 0.1 )
    % while model.withinLimits && any( ref - model.fwdKin > 1e-1 )

        % Compute the optimal control value
        try
            u_star = control( model, histories.u(:,end), ref, 'cartesian' );
            % Note: The finite differences method used to linearize the
            % dynamics may cause joint limitation violation warnings even when
            % the actual posture satisfies the constraints.
        catch
            % The mismatch between linear and nonlinear arm models may cause
            % infeasible states which then result in errors when computing the
            % next u_star.  This catch function allows us to bypass this
            % problem and proceed with computing the rest of the workspace.
            warning(['Optimal control value could not be found, ' ...
                'exiting reach simulation.'])
            break
        end

        % Implement the optimal torques on the model.
        model = plant( model, u_star ); 
        model.draw;


        % Sense the resulting sensory outputs and estimate the next state.

        display(['Time = ' num2str(i*model.Ts) 'sec'])
        display(['Delta q = [' num2str( model.q' - histories.q(:,end)') ...
            ' ]''' ])

        histories.u = [ histories.u, u_star ];
        histories.q = [ histories.q, model.q ];
        histories.y = [ histories.y, fwdKin( model )];
        i = i +1;
        q_diff = diff( histories.q');
    end
end

%% Plot results
time = linspace( 0, model.Ts*length(histories.u(1,:)), ...
    length(histories.u(1,:)));
figure
subplot(3,1,1)
    plot( time, histories.u(1,:), 'b', ...
          time, histories.u(2,:), 'r')
      hold on
      plot( time, ones(size(time))*model.torqLim(1,1), 'b:', ...
            time, ones(size(time))*model.torqLim(1,2), 'b:', ...
            time, ones(size(time))*model.torqLim(2,1), 'r:', ...
            time, ones(size(time))*model.torqLim(2,2), 'r:')
    ylabel 'Optimal joint torques, N-m'
    box off
    
subplot(3,1,2)
    plot( time, histories.q(1,:)*180/pi, 'b', ...
          time, histories.q(2,:)*180/pi, 'r' )
   hold on
  plot( time, ones(size(time))*model.thLim(1,1)*180/pi, 'b:', ...
        time, ones(size(time))*model.thLim(1,2)*180/pi, 'b:', ...
        time, ones(size(time))*model.thLim(2,1)*180/pi, 'r:', ...
        time, ones(size(time))*model.thLim(2,2)*180/pi, 'r:')
    ylabel 'Joint angle trajecotires, degrees'
    box off
subplot(3,1,3)
    plot( time, histories.y(1,:), 'b', ...
          time, histories.y(2,:), 'r' )
    ylabel 'Hand trajectory, m'
    xlabel 'Time (sec)'
    box off
    
figure
P = Polyhedron( histories.y(1:2,:)' );
plot(P)
    box off
    axis equal
    title 'Hand workspace'
    xlabel 'x'
    ylabel 'y'