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
