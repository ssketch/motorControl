clear; clc; close all;
%%% This main function is to be used for the development and testing of
%%% object-oriented implementations of the arm model

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
model.q = zeros(4,1);

% Perform a reach:
for i = 1:10
    
    % Compute the optimal control value
    u_star = control( model, zeros(2,1), [ones(2,1); zeros(2,1)])
    
    % Implement the optimal torques on the model.
    model = plant( model, u_star );
    
    % Sense the resulting sensory outputs and estimate the next state.

end



% The 4 DOF model also works with the same framework:
% Setup the model
model = arm_4DOF(subj);

% Set the inital state
model.q = zeros(8,1);

for i = 1:10
    
    % Compute the optimal control value
    u_star = control( model, zeros(4,1), [ones(3,1); zeros(3,1)])
    
    % Implement the optimal torques on the model.
    model = plant( model, u_star );

end
