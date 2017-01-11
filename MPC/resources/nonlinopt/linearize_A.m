function [ slope, const ] = linearize_A( fx, x, Ts )
% function [ slope, const ] = linearize_A( fx, x, Ts )
% This function linearizes the nonlinear dynamics specified in the function
% handle fx about the state provided, x, with a sampling time of Ts.

% Taylor series linearization:
% f(x) ~ f(a) + f'(a)(x - a)
% f(x) ~ f'(a)*x + [f(a) - f'(a)*a]

% Euler discretization:
% f'(x) ~ [f(x+?x)-f(x)]/?x

% Compute f'(a) for each column:
for i = 1:length(x) % For each column/each component of x
    
    eps = 1e-3;
    x_eps = x;
    x_eps(i) = x(i) + eps; % perturb x by a slight amount

    slope(:,i) = ( fx(x_eps) - fx(x)) ./ eps;

end

I = eye(length(x));
% I(1:length(x)/2, length(x)/2+1:end) = eye(length(x)/2) .* Ts;
slope = I + slope*Ts;

const = (fx(x) - slope * x) * Ts;
