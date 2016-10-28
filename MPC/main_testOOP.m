clear; clc; close all;
%%% This main function is to be used for the development and testing of
%%% object-oriented implementations of the arm model

% Subject characteristics
subj.M = 70;    % kg
subj.H = 1.80;  % meters
subj.hand = 'right';

model = arm_2DOF(subj,[]);

