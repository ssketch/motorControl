% This script sets subject, movement, and control/estimation parameters,
% creates an arm model based on these parameters, then simulates a reach
% using the model.
close all
clear
clc

addpath ./include
addpath ./tbxmanager
addpath ./export_fig

% define subject physical parameters
subj.hand = 'left';
subj.M = 80;
subj.H = 1.6;
subj.thMin = [-70;0];
subj.thMax = [120;170];
subj.torqMin = [-85;-60];
subj.torqMax = [100;75];

% define control/estimation parameters
ctrl.space = 'joint';
ctrl.q = 0.00001;
ctrl.rp = 0.01;
ctrl.rv = 0.01;
ctrl.hrzn = 15;
ctrl.wP = 1;
ctrl.wV = 1e-2;
ctrl.wT = 1e-1;
ctrl.alpha = 1e10;

% define movement parameters
movt.type = 0;
movt.p_i = [0;0.4];
movt.v_i = [0;0];
movt.T = 0.8;
movt.dt = 0.02;
movt.d = 0.5;
movt.th = 25;
movt.Thold = 1.5;
movt.a = 0.3;
movt.b = 0.75;
[movt.ref, reachable] = defineRef(movt, ctrl);
if ~reachable
    fprint('\nPoints in reference trajectory are not in arm workspace.\n')
    return
end

% create and initialize arm model
arm = arm_2DOF(subj, movt);

% define subject neural parameters
biasData.angle = [20 30;40 60;65 80];
biasData.bias = [-2 -6;0 2;3 7];
if size(biasData.angle,2) == size(biasData.bias,2) && ...
        size(biasData.angle,2) == arm.jDOF
    subj.bias = defineBiasFunc(biasData);
else
    fprint('\nInsufficient or mismatched data to define bias function.\n')
    return
end
subj.noise = [0.01;0.01];
subj.Td.act = 0.1;
subj.Td.mod = 0.1;

% when ready, simulate reach with arm
answer = questdlg('Ready to run simulation?', 'Reaching Simulation');
if strcmp(answer,'Yes')
    [results, flag] = simulate(arm, subj, movt, ctrl);
else
    fprintf('Parameters and model have been loaded into workspace.\n')
    fprintf('Call ''simulate.m'' when you are ready.\n\n')
end
