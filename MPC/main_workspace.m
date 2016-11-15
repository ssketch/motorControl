clear; clc; close all;
%%% This script computes the workspace for the simulated arm

%% Step 1: Setup arm and MPC toolbox
% Initialize MPT3 toolbox
addpath( genpath([pwd '/tbxmanager']));

% Subject characteristics
subj.M = 70;    % kg
subj.H = 1.80;  % meters
subj.hand = 'right';

% Define a model.  We'll start with the 2 degree of freedom planar model
model = arm_2DOF(subj);

% Initially, the model will be resting at 0 degrees of shoulder horizontal
% rotation and 0 degrees of elbow flexion.
model.q = [ 45; 45; 0 ; 0] * pi/180;
draw(model);

histories.u = [];
histories.q = [];
histories.x = [];

q_history = [];
x_history = [];

ref = [ 30; 30; 0; 0 ] * pi/180;
ref = [ 0.3; 0.3; 0; 0 ];
% Perform a reach:
for i = 1:500
    
    % Compute the optimal control value
    u_star = control( model, zeros(2,1), ref );
    
    % Implement the optimal torques on the model.
    model = plant( model, u_star );
    
    % Sense the resulting sensory outputs and estimate the next state.

    draw( model);
    display(num2str(i))
    
    histories.u = [ histories.u, u_star ];
    histories.q = [ histories.q, model.q ];
    histories.x = [ histories.x, model.x ];
end

figure
subplot(3,1,1)
    plot( histories.u' )
    ylabel 'Optimal joint torques, N-m'
subplot(3,1,2)
    plot( histories.q'*180/pi )
    ylabel 'Joint angle trajecotires, degrees'
subplot(3,1,3)
    plot( histories.x' )
    ylabel 'Hand trajectory, m'