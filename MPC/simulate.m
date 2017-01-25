% This function simulates a movement with a given arm and internal arm
% model using model-predictive control and Kalman filter-based estimation.
% The input movement 'movt' is a struct with the parameters 'space'
% ('joint','task', or 'force'), 'ref' (a discretized trajectory the arm
% tries to follow in the defined space), and 't' (the associated discrete
% time vector). The output data contains the following:
%
%   t:    time vector [sec]
%   uCmd: commanded joint torques [Nm]
%   qAct: joint angles [deg]
%   qEst: estimated joint angles [deg]
%   xAct: arm state, in joint coordinates [deg,deg/s,Nm]
%   xEst: estimated arm state, in joint coordinates [deg,deg/s,Nm]
%   yAct: arm state, in task coordinates [m,m/s,N]
%   yEst: estimated arm state, in task coordinates [m,m/s,N]

function data = simulate(movt, arm, intModel)

% extract arm parameters
nInputs    = length(arm.u.val);
nJoints    = length(arm.q.val);
nStatesJnt = length(arm.x.val);
nStatesTsk = length(arm.y.val);

% extract movement parameters
n = length(movt.t);

% declare variables to save
toDeg = 180/pi;
u_optTraj = [];             % empty to start
uCmd = zeros(nInputs,n);    % [Nm]
qAct = zeros(nJoints,n);    % [deg]
qEst = zeros(nJoints,n);    % [deg]
xAct = zeros(nStatesJnt,n); % [deg,deg/s]
xEst = zeros(nStatesJnt,n); % [deg,deg/s]
yAct = zeros(nStatesTsk,n); % [m,m/s]
yEst = zeros(nStatesTsk,n); % [m,m/s]

% simulate reach
progBar = waitbar(0,'Simulating reach ... t = ');
for i = 1:n
    
    % display progress of simulation
    waitbar(i/n, progBar, ['Simulating reach ... t = ',num2str(movt.t(i))]);
    
    % save current data
    uCmd(:,i) = arm.u.val; % = intModel.u.val (they receive same input)
    qAct(:,i) = arm.q.val*toDeg;
    qEst(:,i) = intModel.q.val*toDeg;
    xAct(:,i) = arm.x.val;
    xEst(:,i) = intModel.x.val;
    yAct(:,i) = arm.y.val;
    yEst(:,i) = intModel.y.val;
    
    % compute optimal control trajectory (only if enough time has passed)
    if movt.t(i) ~= 0 && mod(movt.t(i),arm.Tr) == 0
        [u_optTraj, flag] = control(intModel, movt.ref(:,i), movt.space);
        if flag
            warning('Linearization failed.')
            return
        end
    end
    
    % set torque to zero if haven't planned that far in advance;
    % otherwise, grab torque from preplanned trajectory
    if isempty(u_optTraj)
        u_opt = zeros(nInputs,1);
    else
        u_opt = u_optTraj(:,1);
    end
    
    % actuate arm with optimal control & sense feedback
    zNext = actuate(arm, u_opt);
    x_sens = sense(arm, zNext);
    
    % estimate current state, storing it in internal model
    estimate(intModel, u_opt, x_sens);

    % discard most recently applied control
    u_optTraj = u_optTraj(:,2:end);
    
end
close(progBar)

% convert angles in state to degrees
xAct(1:2*nJoints,:) = xAct(1:2*nJoints,:)*toDeg;
xEst(1:2*nJoints,:) = xEst(1:2*nJoints,:)*toDeg;

% save data in a struct
data.t = movt.t;
data.uCmd = uCmd;
data.qAct = qAct;
data.qEst = qEst;
data.xAct = xAct;
data.xEst = xEst;
data.yAct = yAct;
data.yEst = yEst;

end