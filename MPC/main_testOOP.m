clear; clc; close all;
%%% This main function is to be used for the development and testing of
%%% object-oriented implementations of the arm model

% Initialize MPT3 toolbox
addpath( genpath([pwd '/tbxmanager']));

% Subject characteristics
subj.M = 70;    % kg
subj.H = 1.80;  % meters
subj.hand = 'right';

model = arm_4DOF(subj);

model.q = zeros(8,1);


% control( model, zeros(4,1), [ones(3,1); zeros(3,1)])


%% 2 DOF test
model = arm_2DOF(subj);

model.q = zeros(4,1);

control( model, zeros(2,1), ones(2,1))