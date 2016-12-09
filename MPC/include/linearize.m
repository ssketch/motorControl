% This function 
function [A, B, C, c, d] = linearize(arm, x_est, space)

%% LINEARIZE
% To linearize the model, we compute the 1st-order Taylor series
% approximation of (1) dynamics dx/dt = f(x,u) & (2) output (e.g., forward
% kinematics) y = g(x), about the current state x_k and most recent control
% u_(k-1). The dynamics will take the (affine) form dx/dt = f(x,u) = Ax +
% Bu + c, where A = df/dx and B = df/du. The output equation will take the
% (affine) form y = g(x) = Cx + d, where C = dg/dx. Both linear
% approximations are only reasonable for x "close" to x_k, which we assume
% to hold over MPC's finite horizon. For example, the linearization of the
% dynamics proceeds as follows:
%
%  f(x,u) = f(x_k,u_k) + df/dx*(x - x_k) + df/du*(u - u_k) + H.O.T.
%  f(x,u) ~ (df/dx)*x + (df/du)*u + [f(x_k,u_k) - (df/dx)*x_k - (df/du)*u)k]
%  f(x,u) ~ Ax + Bu + c
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
A = zeros( length( arm.x.min ), length( arm.x.min ));
B = zeros( length( arm.x.min ), length( arm.u.min ));
if isempty( arm.y )
    warning('Output value, y, not initialized.')
else
    C = eye( length( arm.y.val ), length( arm.x.min ));
end

% compute dynamics matrix, A
f = dynamics(arm, x, u);
for i = 1:length( arm.x.min )
    x_eps = x;
    x_eps(i) = x_eps(i) + eps;       % 1 state perturbed
    f_eps = dynamics(arm, x_eps, u); % state-perturbed dynamics
    A(:,i) = (f_eps-f)/eps;
end

% compute input-to-state matrix, B
for i = 1:length( arm.u.min )
    u_eps = u;
    u_eps(i) = u_eps(i) + eps;           % 1 input perturbed
    f_eps = dynamics(arm, x, u_eps); % input-perturbed dynamics
    B(:,i) = (f_eps-f)/eps;
end

% compute constant vector, c
c = f - A*arm.x.val - B*u;

% compute measurement matrix, C, and constant vector, d
switch space
    
    % output joint-space state
    case 'joint'
        C = eye(nOutputs,nStates);
        d = zeros(nOutputs,nInputs);
    
    % output task-space coordinates by linearizing forward kinematics
    case 'cartesian'
        g = arm.fwdKin;
        C = zeros( length(g), length( x ));

        for i = 1:length(x)
            x_eps = x;
            x_eps(i) = x_eps(i) + eps;  % 1 state perturbed
            g_eps = fwdKin(arm, x_eps); % state-perturbed forward kinematics
            C(:,i) = (g_eps-g)/eps;
        end
        d = g - C*x;
    
    % torque (generalized force) control
    case 'force'
        
        %%%%%%%%%
        % TO DO %
        %%%%%%%%%

    % joint space by default
    otherwise
        C = eye(nOutputs,nStates);
        d = zeros(nOutputs,nInputs);
end

%% DISCRETIZE
%
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

end