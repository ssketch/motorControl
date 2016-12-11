% This function uses sensory feedback in concert with the nonlinear arm
% model to estimate the arm's state, as well as this estimate's
% uncertainty. The unscented Kalman filter algorithm is based on Julier,
% S.J. and Uhlmann, J.K., Unscented Filtering and Nonlinear Estimation,
% Proceedings of the IEEE, Vol. 92, No. 3, pp. 401-422, 2004. The original
% code was written by Yi Cao at Cranfield University in 2008.
function [z_est, P] = ukf(armModel, x_sens, u, f, g)

% define parameters
n = length(armModel.x.val);        % number of states
N = length(armModel.z.val);     % number of (augmented) states
m = length(x_sens);                % number of sensed outputs
alpha = 0.15;                      % default (tunable)
ki = 0;                            % default (tunable)
beta = 2;                     % default (tunable)
lambda = alpha^2*(N+ki) - N); % scaling factor
c = N + lambda;               % scaling factor

% create process and sensing noise matrices, Q and R
Q = zeros(N);
Q(1:n,1:n) = armModel.Ts * diag(armModel.motrNoise*ones(n,1));
R = armModel.Ts * diag(armModel.sensNoise);

% compute weights for means & covariance
Wm = [lambda/c (0.5/c + zeros(1,2*N))];
Wc = Wm;
Wc(1) = Wc(1) + (1-alpha^2+beta);

% compute sigma points around current state
c = sqrt(c);
Z = sigmas(armModel.z.val, armModel.P, c);

% perform unscented transforms
[z, Pz, Z1, Zd] = ut(armModel, f, u, Q, Z,  Wm, Wc, N); % (augmented) plant
[y, Py,  ~, Yd] = ut(armModel, g, u, R, Z1, Wm, Wc, m); % sensing

% update state estimate and covariance
Pzy = Zd*diag(Wc)*Yd';
K = Pzy/Py;
z_est = z + K*(x_sens-y);
P = Pz - K*Pzy';


% This helper function computes the sigma points around a state x given
% covariance P.
function X = sigmas(x, P, c)

if min(eig(P)) < 1e-4
    P = nearestSPD(P); % correct for non-PD matrices before Cholesky
end
A = c*chol(P)';
Y = x(:,ones(1,length(x)));
X = [x Y+A Y-A];


% This helper function performs an unscented transform through the
% nonlinear map f on the sigma points X. It outputs the transformed mean y,
% covariance P, sigma points Y, and deviations (of sigma points from mean)
% Yd.
function [y, P, Y, Yd] = ut(armModel, f, u, cov, X, Wm, Wc, n)

L = size(X,2);  % number of sigma points
y = zeros(n,1);
Y = zeros(n,L);
N = length(armModel.z.val);

% transform each sigma point & compute mean as weighted sum
for k = 1:L
    if n == N
        Y(:,k) = f(armModel, u, X(:,k)); % working with plant
    else
        Y(:,k) = f(armModel, X(:,k));    % working with sensor
    end
    y = y + Wm(k)*Y(:,k);
end

% compute deviations and covariance
Yd = Y - y(:,ones(1,L));
P = Yd*diag(Wc)*Yd' + cov;
