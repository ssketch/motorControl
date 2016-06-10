% This function "linearizes" the 2-DOF arm dynamics about state x and
% control signal T.
function [Ad, Bd, cd, Aext, Bext] = linearize(x, T, params)

% extract current state estimates
th = x(1:2);
th_dot = x(3:4);

% estimate nonlinear, continuous-time equations of motion dx/dt = f(x)
[M,V,~] = dynamics(th, th_dot, 0, params);
f = [th_dot ; M\(T-V-params.B*th_dot)];

% "linearize" equations of motion: dx/dt = Ax + Bu + c, where A = df/dx
A = zeros(params.Nstates);
B = zeros(params.Nstates,params.Nactuat);
eps = 1e-3;
for i = 1:params.Nstates
    x_pert = x;
    x_pert(i) = x(i) + eps;
    [Mpert,Vpert,~] = dynamics(x_pert(1:2), x_pert(3:4), 0, params);
    feps = [x_pert(3:4) ; Mpert\(T-Vpert-params.B*x_pert(3:4))];
    A(:,i) = (feps-f)/eps;
end
for i = 1:params.Nactuat
    T_pert = T;
    T_pert(i) = T(i) + eps;
    feps = [th_dot ; M\(T_pert-V-params.B*th_dot)];
    B(:,i) = (feps-f)/eps;
end
c = f - A*x - B*T;

% convert to discrete time
Ad = A*params.dt + eye(size(A));
Bd = B*params.dt;
cd = c*params.dt;

% extend state
Aext = diag(ones((params.Nstates)*params.numDelSteps_mod,1),-(params.Nstates));
Aext(1:params.Nstates,1:params.Nstates) = Ad;
Bext = [Bd;zeros(params.Nstates*params.numDelSteps_mod,params.Nactuat);...
    zeros(1,params.Nactuat)];
cext = [c;zeros((params.Nstates)*params.numDelSteps_mod,1)];

% convert from affine to purely linear system
Aext = [Aext                   cext;
        zeros(1,size(Aext,2))  1];

end