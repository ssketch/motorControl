% This script provides an example of parameters that must be set prior to
% calling 'plant.m', 'control.m', or 'estimate.m'. Most likely, this code
% will be copied into a block of code at the top of a "main" script.

% define subject parameters (physical)
actualSubj.M = 70;         % mass [kg]
actualSubj.H = 1.80;       % height [meters]
actualSubj.Td = 0.16;      % time delay [sec]
actualSubj.coupled = 0;    % 0 = no joint coupling, 1 = coupling present
actualSubj.C = NaN;        % joint-coupling matrix (only if 'coupled' = 1)
actualSubj.hand = 'right'; % handedness

% define internal model parameters (mental)
modelSubj.M = 70;
modelSubj.H = 1.80;
modelSubj.Td = 0.10;
modelSubj.coupled = 0;
modelSubj.C = NaN;
modelSubj.hand = 'right';

% create arm objects
arm = arm_2DOF(actualSubj);
intModel = arm_2DOF(modelSubj);

% define timing parameters
T = 1;                     % movement time [sec]
movt.Ts = 0.01;            % time step [sec]
movt.t = 0:movt.Ts:movt.T; % time vector [sec]
movt.n = length(movt.t);   % number of time steps

% define movement parameters
d = 0.35;               % reach distance [m]
th = 10;                % reach angle [deg]
Thold = 2;              % time to hold at endpoint [sec]
a = 0.25;               % ellipse semi-major axis [m]
b = 0.1;                % ellipse semi-minor axis [m]

movt.p_i = [-0.15;0.3]; % initial position [m]
movt.v_i = [0;0];       % initial velocity [m/s]
[th_i,th_dot_i,~] = invKin(p.p_i,v_i,[0;0],p); % position & velocity [rad]
p.x0 = [th_i;th_dot_i];                        % state

% reference to be tracked
if (movemnt == 0)
    
    p.p_f = p.p_i + d*[cosd(th);sind(th)];     % desired end position [m]
    v_f = [0;0];                                   % desired end velocity [m/s]
    [th_f,th_dot_f,~] = invKin(p.p_f,v_f,[0;0],p);
    p.y_des = repmat([th_f;th_dot_f],1,p.n);
    
elseif (movemnt == 1)
    
    T = T + Thold;
    p.t = 0:p.dt:T;
    p.n = length(p.t);
    
    p.p_f = p.p_i + d*[cosd(th);sind(th)];
    v_f = [0;0];
    [th_f,th_dot_f,~] = invKin(p.p_f,v_f,[0;0],p);
    p.y_des = repmat([th_f;th_dot_f],1,p.n);
    
elseif (movemnt == 2)
    
    T = 2*T;
    p.t = 0:p.dt:T;
    p.n = length(p.t);
    
    p.p_atTurn = p.p_i + d*[cosd(th);sind(th)];
    v_atTurn = [0;0];
    [th_atTurn,th_dot_atTurn,~] = invKin(p.p_atTurn,v_atTurn,[0;0],p);
    y_atTurn = [th_atTurn;th_dot_atTurn];
    
    p_f = p.p_i;
    v_f = [0;0];
    [th_f,th_dot_f,~] = invKin(p_f,v_f,[0;0],p);
    
    p.y_des = repmat([th_f;th_dot_f],1,p.n);
    p.y_des(:,1:floor(p.n/2)) = repmat(y_atTurn,1,floor(p.n/2));
    
elseif (movemnt == 3)
    
    T = 2*T;
    p.t = 0:p.dt:T;
    p.n = length(p.t);
    
    t_adj = pi + (2*pi/T)*p.t;
    p.x_ell = a*cos(t_adj)*cosd(th) - b*sin(t_adj)*sind(th) + p.p_i(1);
    p.y_ell = a*cos(t_adj)*sind(th) + b*sin(t_adj)*cosd(th) + p.p_i(2);
    
    th_ell = zeros(2,p.n);
    th_dot_ell = zeros(2,p.n);
    for i = 1:p.n
        [th_ell(:,i),th_dot_ell(:,i),~] = ...
            invKin([p.x_ell(i);p.y_ell(i)],[0;0],[0;0],p);
    end
    p.y_des = [th_ell;th_dot_ell];
    
end


% define MPC parameters
Treact = 0.1;              % "reaction time" for replanning torque trajectory (Wagner & Smith, 2008) [sec]
p.Nhrzn = Treact/p.dt + 1; % number of time steps in optimization horizon
ctrl.H = 15;     % prediction horizon

% define UKF parameters
est.