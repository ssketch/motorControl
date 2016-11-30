% This function returns the (joint-space) equation of motion for the arm in
% its current state q, given input u. NOTE: Although the arm's current
% state is an attribute of the arm object, it must be passed as an input
% for use with the MATLAB solver 'ode45'.
function f = dynamics(arm, q, u)

% compute "inertia" parameters
a1 = arm.I1 + arm.I2 + arm.m2*arm.l1^2;
a2 = arm.m2 * arm.l1 * arm.s2;
a3 = arm.I2;

% compute dynamics matrices in joint space
M11 = a1 + 2*a2*cos(q(2));
M12 = a3 + a2*cos(q(2));
M21 = M12;
M22 = a3;
M = [M11 M12;
     M21 M22];

V1 = -q(4)*(2*q(3) + q(4));
V2 = q(3)^2;
V = [V1;V2] * a2*sin(q(2));

% add in gravity and friction
G = [0;0];
Fric = arm.B*q(3:4);

f = [q(3:4) ; M\(u-V-G-Fric)];

end