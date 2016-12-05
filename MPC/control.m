function [u, flag] = control(arm, x_est, u, movt, params)
% This function computes the MPC control output u for a model of the arm
% tracking a reference state ref in either joint or task (Cartesian) space.
% It employs the Multi-Parametric Toolbox MPT3. The MPT model is created by
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
%                |                _______|_____     |
%                |                |           |     |
%                |________________| Estimator |_____|
%                                 |___________|     
%

%% LINEARIZATION
% To linearize the model, we compute the 1st-order Taylor series
% approximation of (1) dynamics dx/dt = f(x,u) and (2) output y = g(x),
% about current state x_k and control u_k. The dynamics will take the
% (affine) form dx/dt = f(x,u) = Ax + Bu + c, where A = df/dx and B =
% df/du. The output equation will take the (affine) form y = g(x) = Cx + d,
% where C = dg/dx. Both linear approximations are only reasonable for x
% "close" to x_k, which we assume to hold over MPC's finite horizon. For
% example, the linearization of the dynamics proceeds as follows:
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

% define small delta for Euler differentiation
eps = 1e-3;

% allocate memory for matrices/vectors
A = zeros(nStates);
B = zeros(nStates,nInputs);
C = eye(nOutputs,nStates);

% compute dynamics matrix, A
f = dynamics(arm, arm.q, u);
for i = 1:nStates
    q_eps = arm.q;
    q_eps(i) = q_eps(i) + eps;       % 1 state perturbed
    f_eps = dynamics(arm, q_eps, u); % state-perturbed dynamics
    A(:,i) = (f_eps-f)/eps;
end

% compute input-to-state matrix, B
for i = 1:nInputs
    u_eps = u;
    u_eps(i) = u_eps(i) + eps;           % 1 input perturbed
    f_eps = dynamics(arm, arm.q, u_eps); % input-perturbed dynamics
    B(:,i) = (f_eps-f)/eps;
end

% compute constant vector, c
c = f - A*arm.q - B*u;

% compute measurement matrix, C, and constant vector, d
switch space
    
    % output joint-space state
    case 'joint'
        C = eye(nOutputs,nStates);
        d = zeros(nOutputs,nInputs);
    
    % output task-space coordinates by linearizing forward kinematics
    case 'cartesian'
        C = zeros(nOutputs,nStates);
        g = fwdKin(arm);
        for i = 1:nStates
            q_eps = arm.q;
            q_eps(i) = q_eps(i) + eps;  % 1 state perturbed
            g_eps = fwdKin(arm, q_eps); % state-perturbed forward kinematics
            C(:,i) = (g_eps-g)/eps;
        end
        d = g - C*arm.q;
        
    % joint space by default
    otherwise
        C = eye(nOutputs,nStates);
        d = zeros(nOutputs,nInputs);
end

%% DISCRETIZATION
% To discretize the model, we use a zero-order hold approximation. This is
% only applied to the dynamics since the output equation is algebraic, not
% differential. The derivation is as follows:
%
%   dx/dt = Ax + Bu + c
%   (x+ - x)/Ts = Ax + Bu + c
%   x+ = (Ax + Bu + c)*Ts + x
%   x+ = (A*Ts+I)x + (B*Ts)u + (c*Ts)
%   x+ = Ad*x + Bd*u + cd

Ad = A*arm.Ts + eye(size(A));
Bd = B*arm.Ts;
cd = c*arm.Ts;

%% OPTIMIZATION
% Optimization is done via quadratic programming, using the Multi-
% Parametric Toolbox MPT3.

% define model
model = LTISystem('A',Ad,'B',Bd,'f',cd,'C',C,'g',d,'Ts',movt.Ts);

% make model track a reference (can be time-varying)
model.y.with('reference');
model.y.reference = 'free';

% set (hard) constraints
model.x.min = arm.jntLim(:,1);
model.x.max = arm.jntLim(:,2);
model.u.min = arm.torqLim(:,1);
model.u.max = arm.torqLim(:,2);

% define cost function
switch movt.space
    case 'joint'
    model.y.penalty = QuadFunction(diag(ctrl.wP*[ones(arm.tDOF,1); ...
        1e-1*ones(arm.tDOF,1)]));
    case 'task'
    model.y.penalty = QuadFunction( diag(1e3*[ones(arm.tDOF,1); ...
        1e-1*ones(arm.tDOF,1)]));
    otherwise
        
end
model.u.penalty = QuadFunction( diag(ones(arm.jDOF,1)));



% create MPC controller
ctrl = MPCController(model, arm.H);

% simulate open-loop system
u = ctrl.evaluate(arm.q, 'y.reference', movt.ref);

% check that optimal control is within bounds
if (Tcurr(1) < params.torq_lim(1,1) || Tcurr(1) > params.torq_lim(1,2)) ...
        || (Tcurr(2) < params.torq_lim(2,1) || Tcurr(2) > params.torq_lim(2,2))
    flag = 1;
    return
end
end        
