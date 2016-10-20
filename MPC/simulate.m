% This function simulates the arm model of the given subject for the
% specified movement using the specified optimal control/estimator
% parameters. The output 'flag' signals success of the simulation: 0 =
% success, 1 = point along desired trajectory outside workspace, 2 = unable
% to find optimal control.
function [results, flag] = simulate(arm, subj, movt, ctrl)

prompt = 'Is the Multi-Parametric Toolbox (MPT) installed? Y/N: ';
MPTinstalled = input(prompt,'s');
if strcmp(MPTinstalled,'N') || strcmp(MPTinstalled,'n')
    fprintf('\nRun ''install_mpt3.m'' then try again.\n')
    return
end

%% PARAMETERS

% dynamical system dimensions
if strcmp(ctrl.space,'joint')
    n = length(arm.q); % number of states (at one time step)
else
    n = length(arm.x);
end
m = arm.jDOF;          % number of inputs (one torque actuator per joint)
p = n;                 % number of outputs (state at one time step)

% declare variables to save
inWS = 1;
Topt = zeros(params.Nactuat,params.n);
xAct = zeros(params.Nstates,params.n);
xEst = zeros(params.Nstates,params.n);

% initialize simulation-level variables
Tcurr = zeros(params.Nactuat,1);
xA = [repmat(params.x0,params.numDelSteps+1,1);1];
xP = [repmat(params.x0,params.numDelSteps_mod+1,1);1];
P = 0.0025*eye(length(xP));

progBar = waitbar(0,'Simulating reach ... t = ');
for i = 1:params.n
    
    % display progress of simulation
    waitbar(i/params.n, progBar, ...
        ['Simulating reach ... t = ',num2str(params.t(i))])
    
    % save data
    Topt(:,i) = Tcurr;
    xAct(:,i) = xA(1:params.Nstates);
    xEst(:,i) = xP(1:params.Nstates);
    
    % linearize about current state, checking for feasibility
    [A, B, f, ~, ~] = linearize(xP(1:params.Nstates), Tcurr, params);
    if ~isreal(A) || sum(sum(isnan(A))) > 0
        isWS = 0;
        break
    end
    
    % build model
    model = LTISystem('A',A, 'B',B, 'f',f, 'C',params.C, 'Ts',params.dt);
    
    % compute control via MPC, checking if it is within bounds
    Tcurr = control(model, xP(1:params.Nstates), params.y_des(:,i), params);
    if (Tcurr(1) < params.torq_lim(1,1) || Tcurr(1) > params.torq_lim(1,2)) ...
            || (Tcurr(2) < params.torq_lim(2,1) || Tcurr(2) > params.torq_lim(2,2))
        isWS = 0;
        break
    end
    
    % actuate arm and sense via proprioception, one step
    xAnext = actuate(xA, Tcurr, params);
    y = sense(xAnext, params);
    
    % estimate next arm state via internal model & Kalman filter
    [xPnext, Pnext] = estimate(xP, P, y, Tcurr, params);
    
    % update simulation-level variables
    xA = xAnext;
    xP = xPnext;
    P = Pnext;
    
end
close(progBar)

% save results in a structure
results.t = params.t;
results.T = Topt;
results.xAct = xAct;
results.xEst = xEst;

% plot and save results
if inWS
    plotResults(results, movt.type, params);
end