function u = control(arm, u, ref, space)
% This function computes the MPC control output u for a model of the arm
% tracking a reference state ref in either joint or Cartesian space. It
% employs the Multi-Parametric Toolbox MPT3. The MPT model is created by
% first linearizing the arm model (both dynamics and output) about the 
% current state estimate.

% ________________________________
% |                              |
% | Control                      |
% |   linearize dx/dt = f(x,u)   |              _________
% |   linearize y = g(x)         |              |       |
% |                              |______________| Plant |
% |   min  y'Qy + u'Ru           |       |      |_______|  
% |   s.t. dx/dt = Ax + Bu       |       |          |
% |        x_min <= x <= x_max   |       |          |
% |        u_min <= u <= u_max   |       |          |
% |______________________________|       |          |
%                |                _______|______    |
%                |                |            |    |
%                |________________|  Estimator |____|
%                                 |____________|     
% 

%% LINEARIZATION
% To linearize the model, compute the 1st-order Taylor series approximation
% of (1) the dynamics dx/dt = f(x,u) and (2) the output y = g(x), about the
% current state x_k and control u_k. The dynamics will take the (affine)
% form dx/dt = f(x,u) = Ax + Bu + c, where A = df/dx and B = df/du. The
% output equation will take the (affine) form y = g(x) = Cx + d, where C =
% dg/dx. Both linear approximations are only reasonable for x ?close? to
% x_k, which we assume to hold over MPC's finite horizon. For example, the
% linearization of the dynamics proceeds as follows:
%
%   f(x,u) = f(x_k,u_k) + df/dx*(x - x_k) + df/du*(u - u_k) + H.O.T.
%   f(x,u) ~ (df/dx)*x + (df/du)*u + [f(x_k,u_k) - (df/dx)*x_k - (df/du)*u)k]
%   f(x,u) ~ Ax + Bu + c
%
% As noted above, A = df/dx and B = df/du in this final equation. Assuming
% small perturbations (dx and du), derivatives are approximated via
% Euler's method:
%
%     df/dx @ x_k ~ [f(x_k+dx)-f(x_k)]/dx
%     df/du @ u_k ~ [f(u_k+du)-f(u_k)]/du

% define constants
nStates = 2*arm.jDOF;
nInputs = arm.jDOF;
nOutputs = numStates;
eps = 1e-3;

% allocate memory for matrices
A = zeros(nStates);
B = zeros(nStates,nInputs);

% compute dynamics matrix, A
f = dynamics(arm, arm.q, u);
for i = 1:nStates
    q_eps = arm.q;
    q_eps(i) = q_eps(i) + eps;      % 1 state perturbed
    feps = dynamics(arm, q_eps, u); % state-perturbed dynamics
    A(:,i) = (feps-f)/eps;
end

% compute input-to-state matrix, B
for i = 1:nInputs
    u_copy = u;
    u_copy(i) = u_copy(i) + eps;         % 1 input perturbed
    feps = dynamics(arm, arm.q, u_copy); % input-perturbed dynamics
    B(:,i) = (feps-f)/eps;
end

% compute measurement matrix, C
switch space
    
    % output joint-space state
    case 'joint'
        C = eye(nOutputs,nStates);
    
    % output task-space coordinates by linearizing forward kinematics
    case 'cartesian'
        
    % joint space by default
    otherwise
        C = eye(nOutputs,nStates);
end
k = fwdKin(arm, arm.q );
for i = 1:nStates
    
    % Perturb each state (joint angle) by a small amount
    qeps = arm.q;
    qeps(i) = qeps(i) + eps;
    
    % Compute the dynamics at the slightly modified state
    feps = fwdKin( arm, qeps );
    C(:,i) = (feps - f) ./ eps;
    
end
y = f - C*arm.q;

% Compute the constant term
c = f - A*arm.q - B*u;


% Step 2) Discretize the linearized model with a zero-order hold
% approximation.

Ad = A*arm.Ts + eye(size(A));
Bd = B*arm.Ts;
cd = c*arm.Ts;

%  I'll leave this part for Shaun....
% % extend state
% Aext = diag(ones((params.Nstates)*params.numDelSteps_mod,1),-(params.Nstates));
% Aext(1:params.Nstates,1:params.Nstates) = Ad;
% Bext = [Bd;zeros(params.Nstates*params.numDelSteps_mod,params.Nactuat);...
%     zeros(1,params.Nactuat)];
% cext = [c;zeros((params.Nstates)*params.numDelSteps_mod,1)];


%% Compute the optimal control for the linearized model
% Define the digital linear model, which looks like this:
%   x_dot = Ax + Bu + f
%   y = Cx + g
% with a time step, Ts.

switch space
    case 'joint'
        % Define the model
        model = LTISystem( 'A', Ad, 'B', Bd, 'f', cd, 'Ts', arm.Ts );
        
        % set constraints
        model.x.min = arm.thLim(:,1);
        model.x.max = arm.thLim(:,2);
        model.u.min = arm.torqLim(:,1);
        model.u.max = arm.torqLim(:,2);
        
        % use soft constraints for feasibility
%         model.x.with('softMax');
%         model.x.with('softMin');
        model.u.with('softMax');
        model.u.with('softMin');
        
        % define cost function
        model.x.penalty = QuadFunction( diag(1e3*[ones(arm.jDOF,1); ...
            1e-2*ones(arm.jDOF,1)]));
        model.u.penalty = QuadFunction( diag(ones(arm.jDOF,1)));

        % make model track a reference (can be time-varying)
        model.x.with('reference');
        model.x.reference = 'free';

        % create MPC controller
        h = 15;     % temporary horizon
        ctrl = MPCController(model, h);

        % simulate open-loop system
        u = ctrl.evaluate( arm.q, 'x.reference', ref);
        
    case 'cartesian'

        %%% Linearize the forward kinematics for tracking
        % This way we effectively get operational space control, but without
        % actually having to do any of the math.  This is because the optimal
        % controller minimizes energetic costs while tracking the end-effector in
        % cartesian space.

        

        % Define the model
        model = LTISystem( 'A', Ad, 'B', Bd, 'f', cd, 'C', C, 'g', y, 'Ts', ...
            arm.Ts );
        
        % set constraints
        model.x.min = arm.thLim(:,1);
        model.x.max = arm.thLim(:,2);
        model.u.min = arm.torqLim(:,1);
        model.u.max = arm.torqLim(:,2);
        
        % use soft constraints for feasibility
        model.x.with('softMax');
        model.x.with('softMin');
        model.u.with('softMax');
        model.u.with('softMin');
        
        % define cost function
        model.y.penalty = QuadFunction( diag(1e3*[ones(arm.tDOF,1); ...
            1e-2*ones(arm.tDOF,1)]));
        model.u.penalty = QuadFunction( diag(ones(arm.jDOF,1)));

        % make model track a reference (can be time-varying)
        model.y.with('reference');
        model.y.reference = 'free';

        % create MPC controller
        h = 15;     % temporary horizon
        ctrl = MPCController(model, h);

        % simulate open-loop system
        u = ctrl.evaluate( arm.q, 'y.reference', ref);
        
    otherwise
        warning('Control space not found.')
end        
