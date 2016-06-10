% This function uses the fully nonlinear dynamics to actuate the arm. The
% state is extended to account for time delay in sensory feedback.
function xNext = actuate(x, T, params)

% determine whether working with real arm or arm model
model = floor(length(x)/params.Nstates) == params.numDelSteps_mod;

% extract current state
th = x(1:2);
th_dot = x(3:4);

% actuate arm using nonlinear, continuous-time equations of motion
[M,V,~] = dynamics(th, th_dot, 0, params);
%f = [th_dot ; M\(T-V-params.B*th_dot)];
if ~model
    coupling = [1 0;0.5 0.5];
    f = [th_dot ; M\(coupling*T-V-params.B*th_dot)];
else
    f = [th_dot ; M\(T-V-params.B*th_dot)];
end

xNext = x;
xNext(1:params.Nstates) = x(1:params.Nstates) + f*params.dt;
if ~model
    xNext(1:params.Nstates) = xNext(1:params.Nstates) + ...
        sqrt(diag(params.Q)).*randn(params.Nstates,1);
end

% time shift delayed states
if model
    delSteps = params.numDelSteps_mod;
else
    delSteps = params.numDelSteps;
end
Mprop = diag(ones((params.Nstates)*delSteps,1),-(params.Nstates));
xNext(1:end-1) = xNext(1:end-1) + Mprop*xNext(1:end-1);

end