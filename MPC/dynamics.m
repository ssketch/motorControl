% This function returns the mass matrix and centrifugal/Coriolis terms for
% the arm's equations of motion, either in joint or Cartesian space. It
% also returns (if desired) the pseudo kinetic-energy matrix for
% description of how the hand would translate in response to a force in its
% current location (most mass in direction of semi-major axis, least mass
% in direction of semi-minor axis).
function [M, V, KE] = dynamics(th, th_dot, inCartesian, params)

th1 = th(1);  th1_dot = th_dot(1);
th2 = th(2);  th2_dot = th_dot(2);

a1 = params.I1 + params.I2 + params.m2*params.l1^2;
a2 = params.m2 * params.l1 * params.s2;
a3 = params.I2;

% compute dynamics matrices in joint space
M11 = a1 + 2*a2*cos(th2);
M12 = a3 + a2*cos(th2);
M21 = M12;
M22 = a3;
M = [M11 M12;
     M21 M22];

V1 = -th2_dot*(2*th1_dot + th2_dot);
V2 = th1_dot^2;
V = [V1;V2] * a2*sin(th2);

% compute pseudo kinetic-energy matrix
J = jacobian(th, params);
KE = J*(M\J');

% convert dynamics matrices to Cartesian space
if inCartesian
    J = jacobian(th, params);
    J_dot = jacobianDeriv(th, th_dot, params);
    Mx = J'\(M/J);
    Vx = J'\(V - (M/J)*J_dot*th_dot);
    M = Mx;
    V = Vx;
end

end