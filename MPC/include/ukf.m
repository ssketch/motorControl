% This function uses sensory feedback from the proprioceptive system in
% concert with the full nonlinear dynamics to estimate the arm's state, as
% well as this estimate's uncertainty. The unscented Kalman filter
% algorithm is based on Julier, S.J. and Uhlmann, J.K., Unscented Filtering
% and Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3, pp.
% 401-422, 2004. The original code was written by Yi Cao at Cranfield
% University in 2008.
function [xPnext, Pnext] = ukf(xP, P, y, T, f, h, params)

% define parameters
L = numel(xP);             % number of states
m = numel(y);              % number of measurements
alpha = 0.15;              % default (tunable)
ki = 0;                    % default (tunable)
beta = 2;                  % default (tunable)
lambda = alpha^2*(L+ki)-L; % scaling factor
c = L+lambda;              % scaling factor

% compute weights for means & covariance
Wm = [lambda/c 0.5/c+zeros(1,2*L)];
Wc = Wm;
Wc(1) = Wc(1)+(1-alpha^2+beta);

% compute sigma points around current state
c = sqrt(c);
X = sigmas(xP, P, c);

% perform unscented transforms
Qext = zeros(L);
Qext(1:params.Nstates,1:params.Nstates) = params.Q;
[x1, X1, P1, X2] = ut(f, X,  Wm, Wc, L, Qext, T, params);
[z1,  ~, P2, Z2] = ut(h, X1, Wm, Wc, m, params.R, T, params);

% update state estimate and covariance
P12 = X2*diag(Wc)*Z2';
K = P12/P2;
xPnext = x1 + K*(y-z1);
Pnext = P1 - K*P12';


% This helper function computes the sigma points around a state x.
function X = sigmas(x, P, c)

if min(eig(P)) < 1e-4
    P = nearestSPD(P); % correct for non-PD matrices before Cholesky
end
A = c*chol(P)';
Y = x(:,ones(1,numel(x)));
X = [x Y+A Y-A];

% This helper function performs an unscented transform on the sigma points.
function [y, Y, P, Y1] = ut(f, X, Wm, Wc, n, cov, T, params)

L = size(X,2);
y = zeros(n,1);
Y = zeros(n,L);

for k = 1:L
    if length(cov) == params.Nsensed
        Y(:,k) = f(X(:,k), params);
    else
        Y(:,k) = f(X(:,k), T, params);
    end
    y = y + Wm(k)*Y(:,k);
end

Y1 = Y - y(:,ones(1,L));
P = Y1*diag(Wc)*Y1' + cov;
