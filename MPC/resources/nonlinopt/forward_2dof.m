function xnext = forward_2dof( x, T, params, sim )

% Compute mass matrix
a1 = params.I1 + params.I2 + params.m2*params.L1^2;
a2 = params.m2 * params.L1 * params.Lc2;
a3 = params.I2;

M11 = a1 + 2*a2*cos(x(2));
M12 = a3 + a2*cos(x(2));
M21 = M12;
M22 = a3;
M = [M11 M12;
     M21 M22];
 
% Compute coriolis and centripetal effects
V1 = -x(4)*(2*x(3) + x(4));
V2 = x(3)^2;
V = [V1;V2] * a2*sin(x(2));

% % Compute Jacobian
% J = [(-params.L1*sind(x(1)) - params.L2*sind(x(1)+x(2))) -params.L2*sind(x(1)+x(2));
%      (params.L1*cosd(x(1)) + params.L2*cosd(x(1)+x(2)))   params.L2*cosd(x(1)+x(2))];


% Update state with a 4th order Runge kutta
f = @(x)  [x(3:4); M\(T-V-(params.B)*x(3:4)) ];
k1 = x + sim.Ts * f( x );
k2 = x + sim.Ts/2 * f( x + sim.Ts/2 * k1 );
k3 = x + sim.Ts/2 * f( x + sim.Ts/2 * k2 );
k4 = x + sim.Ts * f( x + sim.Ts * k3 );

xnext = x + sim.Ts/2 * ( k1 + 2*k2 + 2*k3 + k4 );

% xnext = x + [ x(3:4); M\(T-V-(params.B)*x(3:4)) ] * sim.Ts;
