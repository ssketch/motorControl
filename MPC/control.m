% This function computes the MPC control output u for an MPT model of
% the human arm tracking a reference state provided by ref and the current
% state of the system y.
function u = control(arm, u)

%% Linearize the dynamics of the system about the current state:
% Step 1) Compute a 1st order Taylor series approximation of the dynamics
% method of the model arm object using the method of finite differences.
% The result will be that dq/dt = f(q,u) will take the form dq/dt = Aq + Bu
% + c, where A = df/dq and B = df/du.

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
% This way we effectively get operational space control, but without actually
% having to do any of the math.






%% Compute the optimal control for the linearized model
% % set constraints
% model.x.min = params.xMin;
% model.x.max = params.xMax;
% model.u.min = params.uMin;
% model.u.max = params.uMax;
% 
% % define cost function
% model.x.penalty = QuadFunction(params.xCost);
% model.u.penalty = QuadFunction(params.uCost);
% 
% % make model track a reference (can be time-varying)
% model.x.with('reference');
% model.x.reference = 'free';
% 
% % create MPC controller
% ctrl = MPCController(model, ctrl.hrzn);
% 
% % simulate open-loop system
% u = ctrl.evaluate(y, 'y.reference', movt.ref);
