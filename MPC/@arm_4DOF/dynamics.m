% This function returns the equation of motion for the arm in its current
% state, represented in either joint or task space. In joint space, it is
% of the form q_dot = f(q,u) where u is the vector of joint torques. In
% task space, it is of the form x_dot = f(x,u) where u is the vector of
% hand forces.
function f = dynamics(arm, u, ctrlSpace)

% Compute mass matrix:
M11 = arm.I2*arm.l2^2 + arm.I4+arm.L2^2 + arm.L2^2*arm.m1 + ...
    arm.l2^2*arm.m2 + arm.M2*arm.s1^2

% compute "inertia" parameters
a1 = arm.I1 + arm.I2 + arm.m2*arm.l1^2;
a2 = arm.m2 * arm.l1 * arm.s2;
a3 = arm.I2;

% compute dynamics matrices in joint space
M11 = a1 + 2*a2*cos(arm.q(2));
M12 = a3 + a2*cos(arm.q(2));
M21 = M12;
M22 = a3;
M = [M11 M12;
     M21 M22];

V1 = -arm.q(4)*(2*arm.q(3) + arm.q(4));
V2 = arm.q(3)^2;
V = [V1;V2] * a2*sin(arm.q(2));

G = [0;0];

Fric = arm.B*arm.q(3:4);

% if necessary convert dynamics matrices to task space
if strcmp(ctrlSpace,'joint')
    f = [arm.q(3:4) ; M\(u-V-G-Fric)];
else
    J = arm.jacobian();
    J_dot = arm.jacobianDeriv();
    Mx = J'\(M/J);
    Vx = J'\(V - (M/J)*J_dot*arm.q(3:4));
    Gx = J'\G;
    Fricx = J'\Fric;
    f = [arm.x(3:4) ; Mx\(u-Vx-Gx-Fricx)];
end

end