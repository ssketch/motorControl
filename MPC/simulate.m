

% declare variables to save
u_traj = zeros(params.Nactuat,params.n);
x_traj = zeros(params.Nstates,params.n);
x_est_traj = zeros(params.Nstates,params.n);

progBar = waitbar(0,'Simulating reach ... t = ');
for i = 1:params.n
    
    % display progress of simulation
    waitbar(i/params.n, progBar, ...
        ['Simulating reach ... t = ',num2str(params.t(i))])
    
    % save data
    u_traj(:,i) = arm.u.val;
    x_traj(:,i) = arm.x.val;
    x_est_traj(:,i) = intModel.(1:params.Nstates);
    
    % run through loop
    
    
end
close(progBar)

% save results in a structure
results.t = params.t;
results.T = u_traj;
results.xAct = x_traj;
results.xEst = x_est_traj;

% plot and save results
plotResults(results, movt.type, params);
