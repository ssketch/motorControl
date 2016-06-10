clear all
close all
clc

% define params for simulation
subj.numJoints = 2;
subj.M = 80;
subj.H = 1.6;
subj.healthy = 1;

movt.type = 0;
movt.p_i = [0;0.4];
movt.T = 0.8;
movt.dt = 0.02;
movt.d = 0.5;
movt.th = 25;
movt.Thold = 1.5;
movt.a = 0.3;
movt.b = 0.75;

% Kalman filter noise values etc.
estm.q = 0.00001;
estm.rp1 = 0.01;
estm.rp2 = 0.01;
estm.rv = 0.001;

% Optimal control weights
optm.wP = 1;
optm.wV = 1e-2;
optm.wT = 1e-1;
optm.alpha = 1e10;

% simulate reach
[results, inWS] = simulate(subj, movt, estm, optm);
