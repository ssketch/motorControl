% Given an arm state in joint space, this function checks which joints are
% in the "reflex zone" and, if necessary, computes reflex torques. 'R' is a
% selector matrix, able to extract the appropriate reflex parameters for
% each joint (e.g., the gamma for flexion if the joint is in flexion). If
% no state is specified as input, the function checks the current state of
% the 'arm' object.
function uReflex = computeReflex(arm, R, x)

% if no state specified, use current arm state
if nargin < 3
    x = arm.x.val;
end

% check agreement between arm DOFs and size of R
nJoints = length(arm.q.val);
if size(R,1) ~= nJoints
    error('Dimension mismatch in "inReflexZone.m".')
end

% extract reflex parameters
gamma = R*arm.gamma;
mu = R*arm.mu;
k = R*arm.k;
b = R*arm.b;

% extract joint positions & velocities from arm state
q = x(1:nJoints);
qdot = x(nJoints+1:2*nJoints);

% compare actual and threshold angular deviations
dq = abs(q - arm.q0);
dq_thresh = mu.*qdot + gamma;
reflexive = (dq > dq_thresh);

% compute reflex torques
uReflex = (k.*(arm.q0 - q) + b.*qdot).*reflexive;

end