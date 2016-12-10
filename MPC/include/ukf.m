% This function uses sensory feedback from the proprioceptive system in
% concert with the full nonlinear model to estimate the arm's state, as
% well as this estimate's uncertainty. The unscented Kalman filter
% algorithm is based on Julier, S.J. and Uhlmann, J.K., Unscented Filtering
% and Nonlinear Estimation, Proceedings of the IEEE, Vol. 92, No. 3, pp.
% 401-422, 2004. The original code was written by Yi Cao at Cranfield
% University in 2008.
function ukf(armModel, x_sens, u, f, g)

% define parameters
L = length(armModel.z.val); % number of (augmented) states
m = length(x_sens);         % number of sensed measurements
alpha = 0.15;               % default (tunable)
ki = 0;                     % default (tunable)
beta = 2;                   % default (tunable)
lambda = alpha^2*(L+ki)-L;  % scaling factor
c = L+lambda;               % scaling factor

% compute weights for means & covariance
Wm = [lambda/c (0.5/c + zeros(1,2*L))];
Wc = Wm;
Wc(1) = Wc(1) + (1-alpha^2+beta);

% compute sigma points around current state
c = sqrt(c);
Z = sigmas(armModel.z.val, armModel.P, c);

% create process and sensing noise matrices, Q and R
nStates = length(armModel.x.val);
Q = zeros(L);
Q(1:nStates,1:nStates) = armModel.Ts * diag(armModel.motrNoise*ones(nStates,1));
R = armModel.Ts * diag(armModel.sensNoise);

% perform unscented transforms
[z1, Z1, P1, Z2] = ut(f, Z,  Wm, Wc, L, Q, u); % process
[y1,  ~, P2, Y2] = ut(g, Z1, Wm, Wc, m, R, u); % sensing

% update state estimate and covariance
P12 = Z2*diag(Wc)*Y2';
K = P12/P2;
armModel.z.val = z1 + K*(x_sens-y1);
armModel.P = P1 - K*P12';


% This helper function computes the sigma points around a state x.
function X = sigmas(x, P, c)

if min(eig(P)) < 1e-4
    P = nearestSPD(P); % correct for non-PD matrices before Cholesky
end
A = c*chol(P)';
Y = x(:,ones(1,numel(x)));
X = [x Y+A Y-A];

% This helper function performs an unscented transform on the sigma points.
function [y, Y, P, Y1] = ut(f, X, Wm, Wc, n, cov, u)

L = size(X,2);
y = zeros(n,1);
Y = zeros(n,L);

for k = 1:L
    if length(cov) == params.Nsensed
        Y(:,k) = f(X(:,k), params);
    else
        Y(:,k) = f(X(:,k), u, params);
    end
    y = y + Wm(k)*Y(:,k);
end

Y1 = Y - y(:,ones(1,L));
P = Y1*diag(Wc)*Y1' + cov;
