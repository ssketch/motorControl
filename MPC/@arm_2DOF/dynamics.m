% This function returns the (joint-space) equation of motion for the arm in
% state x, given input u. If no state is specified as input, the function
% computes the equation for the current state of the 'arm' object. The
% function also saves the reflexive torques to the input arm object.
% NOTE: Although the arm's current state is an attribute of the arm object,
% ----  it must be passed as an input for use with the MATLAB solver
%       'ode45'.
function f = dynamics(arm, x, u)

% if no input is specified, use stored value
if nargin < 3
    u = arm.u.val;
    
    % if no state is specified, use current arm state
    if nargin < 2
        x = arm.x.val;
    end
end

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

% couple current joint torques
uCouple = arm.coupling*x(5:8);

% account for reflex activity (NOTE: because of how joint zeros are set,
% arm segment is always moving "away" from its zero point)
if (x(3) > 0)
    if (x(4) > 0) R = [0, 1, 0, 0; 0, 0, 0, 1]; % S flex, E flex
    else          R = [0, 1, 0, 0; 0, 0, 1, 0]; % S flex, E ext
    end
else
    if (x(4) > 0) R = [1, 0, 0, 0; 0, 0, 0, 1]; % S ext, E flex
    else          R = [1, 0, 0, 0; 0, 0, 1, 0]; % S ext, E ext
    end
end
uReflex = computeReflex(arm, R, x);

% low-pass filter commanded joint torques
uLoPass = (u - x(5:8))/arm.tau;

% save reflex torques & output equation of motion
arm.uReflex = uReflex;
f = [x(3:4) ; M\(uCouple+uReflex-V-G-Damp) ; uLoPass];

end