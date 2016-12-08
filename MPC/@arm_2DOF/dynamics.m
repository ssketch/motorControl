% This function returns the (joint-space) equation of motion for the arm in
% state x, given input u. If no state is specified as input, the function
% computes the equation for the current state of the 'arm' object.
% NOTE: Although the arm's current state is an attribute of the arm object,
% ----  it must be passed as an input for use with the MATLAB solver
%       'ode45'.
function f = dynamics(arm, x, u)

% if no input is specified (and there is one stored in the model), use
% stored value
if nargin < 3 && ~isempty(arm.u.val)
    x = arm.x.val;
    
    % if no state is specified, use current arm state
    if nargin == 1
        u = arm.u.val;
    end
end

% couple joint torques
uCouple = arm.coupling*u;

% compute "inertia" parameters
a1 = arm.I1 + arm.I2 + arm.m2*arm.l1^2;
a2 = arm.m2 * arm.l1 * arm.s2;
a3 = arm.I2;

% compute dynamics matrices in joint space
M11 = a1 + 2*a2*cos(x(2));
M12 = a3 + a2*cos(x(2));
M21 = M12;
M22 = a3;
M = [M11 M12;
     M21 M22];

V1 = -x(4)*(2*x(3) + x(4));
V2 = x(3)^2;
V = [V1;V2] * a2*sin(x(2));

% add in gravity and viscous damping
G = [0;0];
Damp = arm.B*x(3:4);

f = [x(3:4) ; M\(uCouple-V-G-Damp)];

end