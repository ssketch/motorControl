% This function simulates a movement with a given arm and internal arm
% model using model-predictive control and Kalman filter-based estimation.
% The input movement 'movt' is a struct with the parameters 'space'
% ('joint','task', or 'force'), 'ref' (a discretized trajectory the arm
% tries to follow in the defined space), and 't' (the associated discrete
% time vector). The output data contains the following:
%
%   t:    time vector [sec]
%   u:    joint torques [Nm]
%   qAct: joint angles [deg]
%   xAct: arm state, in joint coordinates [deg,deg/s]
%   yAct: arm state, in task coordinates [m,m/s]
%   qEst: estimated joint angles [deg]
%   xEst: estimated arm state, in joint coordinates [deg,deg/s]
%   yEst: estimated arm state, in task coordinates [m,m/s]

function data = simulate(movt, arm, intModel)

% extract arm parameters
nInputs    = length(arm.u.val);
nJoints    = length(arm.q.val);
nStatesJnt = length(arm.x.val);
nStatesTsk = length(arm.y.val);

% extract movement parameters
n = length(movt.t);

% declare variables to save
u    = zeros(nInputs,n);    % [Nm]
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
    u(:,i)    = arm.u.val; % = intModel.u.val (they receive same input)
    qAct(:,i) = arm.q.val;
    qEst(:,i) = intModel.q.val;
    xAct(:,i) = arm.x.val;
    xEst(:,i) = intModel.x.val;
    yAct(:,i) = arm.y.val;
    yEst(:,i) = intModel.y.val;
    
    % compute optimal control trajectory (only if enough time has passed)
    if mod(movt.t(i),arm.Tr) == 0
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

% save data in a struct
toDeg = 180/pi;
data.t = movt.t;
data.u = u;
data.qAct = qAct*toDeg;
data.qEst = qEst*toDeg;
data.xAct = xAct*toDeg;
data.xEst = xEst*toDeg;
data.yAct = yAct;
data.yEst = yEst;

end