% This function uses sensory feedback in concert with the full nonlinear
% arm model to estimate the arm's state, as well as this estimate's
% uncertainty. The unscented Kalman filter algorithm is based on Julier,
% S.J. and Uhlmann, J.K., Unscented Filtering and Nonlinear Estimation,
% Proceedings of the IEEE, Vol. 92, No. 3, pp. 401-422, 2004. The original
% code was written by Yi Cao at Cranfield University in 2008.
function [z_est, P] = ukf(armModel, x_sens, u, f, g)

% define parameters
nStates = length(armModel.x.val);              % number of states
nStatesAug = length(armModel.z.val);           % number of (augmented) states
nOutputs = length(x_sens);                     % number of sensed outputs
alpha = 0.15;                                  % default (tunable)
ki = 0;                                        % default (tunable)
beta = 2;                                      % default (tunable)
lambda = alpha^2*(nStatesAug+ki) - nStatesAug; % scaling factor
c = nStatesAug+lambda;                         % scaling factor

% compute weights for means & covariance
Wm = [lambda/c (0.5/c + zeros(1,2*nStatesAug))];
Wc = Wm;
Wc(1) = Wc(1) + (1-alpha^2+beta);

% compute sigma points around current state
c = sqrt(c);
Z = sigmas(armModel.z.val, armModel.P, c);

% create process and sensing noise matrices, Q and R
Q = zeros(nStatesAug);
Q(1:nStates,1:nStates) = armModel.Ts * diag(armModel.motrNoise*ones(nStates,1));
R = armModel.Ts * diag(armModel.sensNoise);

% perform unscented transforms
[z1, Z1, P1, Z2] = ut(armModel, f, Z,  Wm, Wc, nStatesAug, Q, u); % (augmented) plant
[y1,  ~, P2, Y2] = ut(armModel, g, Z1, Wm, Wc, nOutputs,   R, u); % sensing

% update state estimate and covariance
P12 = Z2*diag(Wc)*Y2';
K = P12/P2;
z_est = z1 + K*(x_sens-y1);
P = P1 - K*P12';


% This helper function computes the sigma points around a state x.
function X = sigmas(x, P, c)

if min(eig(P)) < 1e-4
    P = nearestSPD(P); % correct for non-PD matrices before Cholesky
end
A = c*chol(P)';
Y = x(:,ones(1,length(x)));
X = [x Y+A Y-A];


% This helper function performs an unscented transform on the sigma points.
function [y, Y, P, Y1] = ut(armModel, f, X, Wm, Wc, n, cov, u)

nStatesAug = length(armModel.z.val);
L = size(X,2);
y = zeros(n,1);
Y = zeros(n,L);

for k = 1:L
    if length(cov) == nStatesAug
        Y(:,k) = f(X(:,k));       % working with plant
    else
        Y(:,k) = f(X(:,k), u);    % working with sensor
    end
    y = y + Wm(k)*Y(:,k);
end

Y1 = Y - y(:,ones(1,L));
P = Y1*diag(Wc)*Y1' + cov;
