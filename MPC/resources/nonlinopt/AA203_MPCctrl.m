function u = AA203_MPCctrl( x, u, ref, params, sim )  
% Cole Simpson & Sean Sketch
% This function computes the MPC control output for a nonlinear model of
% the human arm tracking a reference state provided by ref, the current
% state of the system (x), the last used control value (u, use [0, 0]' if
% desired), and model and simulation parameters provided in params and sim,
% respectively.  

% Linearize nonlinear model about current state
[ A, B, f ] = linearize_xdot(@(x1,u1) forward_dyn(x1,u1,params), x, u, ...
    sim.Ts );
[ C, D, g ] = linearize_xdot(@(x1,u1) forward_dyn(x1,u1,params), x, u, ...
    sim.Ts );
model = LTISystem( 'A', A, 'B', B, 'f', f, 'C', C, 'D', D, 'g', g, ...
    'Ts', sim.Ts );

% State constraints
model.x.min = [ params.th1Min, params.th2Min, -inf, -inf ]' * pi/180;
model.x.max = [ params.th1Max, params.th2Max,  inf,  inf ]' * pi/180;
% Controller constraints
model.u.min = [ params.torq1Min, params.torq2Min ];
model.u.max = [ params.torq1Max, params.torq2Max ];
% Cost function parameter weightings
model.x.penalty = QuadFunction( 1e3*diag([ 1, 1, 1e-2, 1e-2 ]));
model.u.penalty = QuadFunction( eye(2) );
% Make model track a reference (Can be a time-varying reference)
model.x.with('reference');
model.x.reference = 'free';

% Create MPC controller
ctrl = MPCController(model, sim.N);

% Simulate the closed-loop system
u = ctrl.evaluate( x, 'x.reference', ref);







function [ A, B, f ] = linearize_xdot( fxu, x, u, Ts )
% function [ slope, const ] = linearize_A( fx, x, Ts )
% This function linearizes the nonlinear dynamics specified in the function
% handle fx about the state provided, x, with a sampling time of Ts.

% Taylor series linearization:
% f(x) ~ f(a) + f'(a)(x - a)
% f(x) ~ f'(a)*x + [f(a) - f'(a)*a]

% Euler discretization:
% f'(x) ~ [f(x+?x)-f(x)]/?x

% Compute discrete partial derivative of f(x,u) wrt x
for i = 1:length(x) % For each column/each component of x
    
    eps = 1e-3;
    x_eps = x;
    x_eps(i) = x(i) + eps; % perturb x by a slight amount

    mx(:,i) = ( fxu(x_eps,u) - fxu(x,u)) ./ eps;

end

% Compute discrete partial derivative of f(x,u) wrt u
for i = 1:length(u) % For each column/each component of x
    
    eps = 1e-3;
    u_eps = u;
    u_eps(i) = u(i) + eps; % perturb x by a slight amount

    mu(:,i) = ( fxu(x,u_eps) - fxu(x,u)) ./ eps;

end

A = eye(length(x)) + mx*Ts;
B = mu*Ts;
f = (fxu(x,u) - mx * x - mu*u) * Ts;






function [ A, B, f ] = linearize_y( fxu, x, u )
% function [ slope, const ] = linearize_A( fx, x, Ts )
% This function linearizes the nonlinear dynamics specified in the function
% handle fx about the state provided, x, with a sampling time of Ts.

% Taylor series linearization:
% f(x) ~ f(a) + f'(a)(x - a)
% f(x) ~ f'(a)*x + [f(a) - f'(a)*a]

% Euler discretization:
% f'(x) ~ [f(x+?x)-f(x)]/?x

% Compute discrete partial derivative of f(x,u) wrt x
for i = 1:length(x) % For each column/each component of x
    
    eps = 1e-3;
    x_eps = x;
    x_eps(i) = x(i) + eps; % perturb x by a slight amount

    mx(:,i) = ( fxu(x_eps,u) - fxu(x,u)) ./ eps;

end

% Compute discrete partial derivative of f(x,u) wrt u
for i = 1:length(u) % For each column/each component of x
    
    eps = 1e-3;
    u_eps = u;
    u_eps(i) = u(i) + eps; % perturb x by a slight amount

    mu(:,i) = ( fxu(x,u_eps) - fxu(x,u)) ./ eps;

end

A = mx;
B = mu;
% f = fxu(x,u) - mx*x - mu*u;
f = (fxu(x,u) - mx * x - mu*u) ;

