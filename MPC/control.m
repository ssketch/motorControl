% This function computes the MPC control output u for an MPT model of
% the human arm tracking a reference state provided by ref and the current
% state of the system y.
function u = control(arm, u, ref)

% ________________________________
% | Control                      |
% |   linearize x_dot = F(x,u)   |              _________
% |   linearize y = G(x)         |              |       |
% |                              |______________| Plant |
% |   min y'Qy + u'Ru            |       |      |_______|  
% |   s.t. x_dot = Ax + Bu       |       |          |
% |        x_min <= x <= x_max   |       |          |
% |        u_min <= u <= u_max   |       |          |
% |______________________________|       |          |
%                |                _______|______    |
%                |                |            |    |
%                |________________|  Estimator |____|
%                                 |____________|     
% 

%% Linearize the dynamics of the system about the current state:
% Step 1) Compute a 1st order Taylor series approximation of the dynamics
% method of the model arm object using the method of finite differences.
% The result will be that dq/dt = f(q,u) will take the form dq/dt = Aq + Bu
% + c, where A = df/dq and B = df/du.

% Taylor series linearization:
% f(x) ~ f(a) + f'(a)(x - a)
% f(x) ~ f'(a)*x + [f(a) - f'(a)*a]

% Euler discretization:
% f'(x) ~ [f(x+dx)-f(x)]/dx

% Define some helpful constants
nStates = length( arm.q );
nActuators = length( arm.torqLim );

% Allocate memory for the matrices to be used.
A = zeros(nStates);
B = zeros(nStates, nActuators);
eps = 1e-3;

% Compute the A matrix
f = dynamics( arm, u );
for i = 1:nStates
    
    % Make a copy of the arm
    arm_copy = arm;
    
    % Perturb each state (joint angle) by a small amount
    arm_copy.q(i) = arm_copy.q(i) + eps;
    
    % Compute the dynamics at the slightly modified state
    feps = dynamics( arm_copy, u );
    A(:,i) = (feps-f)/eps;
    
end

% Compute the B matrix
for i = 1:nActuators
    
    % Make a copy of the control inputs (joint torques)
    u_copy = u;

    % Perturb each control input (joint torque) by a small amount
    u_copy(i) = u_copy(i) + eps;
    
    % Compute the dynamics at the slightly modified state
    feps = dynamics( arm, u_copy );
    B(:,i) = (feps-f)/eps;
    
end

% Compute the constant term
c = f - A*arm.q - B*u;


% Step 2) Discretize the linearized model with a zero-order hold
% approximation.

Ts = 0.01;  % For now I'll define this here.  We can figure out what's best as we go.

Ad = A*Ts + eye(size(A));
Bd = B*Ts;
cd = c*Ts;

%  I'll leave this part for Shaun....
% % extend state
% Aext = diag(ones((params.Nstates)*params.numDelSteps_mod,1),-(params.Nstates));
% Aext(1:params.Nstates,1:params.Nstates) = Ad;
% Bext = [Bd;zeros(params.Nstates*params.numDelSteps_mod,params.Nactuat);...
%     zeros(1,params.Nactuat)];
% cext = [c;zeros((params.Nstates)*params.numDelSteps_mod,1)];



%% Linearize the forward kinematics for tracking
% This way we effectively get operational space control, but without
% actually having to do any of the math.  This is because the optimal
% controller minimizes energetic costs while tracking the end-effector in
% cartesian space.

f = fwdKin( arm, arm.q );
for i = 1:nStates
       
    % Perturb each state (joint angle) by a small amount
    q = arm.q;
    q(i) = q(i) + eps;
    
    % Compute the dynamics at the slightly modified state
    feps = fwdKin( arm, q );
    C(:,i) = (feps-f)/eps;
    
end
y = f - C*arm.q;


%% Compute the optimal control for the linearized model
% define linear model
model = LTISystem( 'A', A, 'B', B, 'f', c, 'C', C, 'g', y, 'Ts', 0.01 );

% set constraints
model.x.min = arm.thLim(:,1);
model.x.max = arm.thLim(:,2);
model.u.min = arm.torqLim(:,1);
model.u.max = arm.torqLim(:,2);

% define cost function
model.x.penalty = QuadFunction( diag([ones(arm.jDOF,1); zeros(arm.jDOF,1)]));
model.u.penalty = QuadFunction( diag(ones(arm.tDOF,1)));

% make model track a reference (can be time-varying)
model.y.with('reference');
model.y.reference = 'free';

% create MPC controller
h = 15;     % temporary horizon
ctrl = MPCController(model, h);

% simulate open-loop system
u = ctrl.evaluate( arm.q, 'y.reference', ref);
