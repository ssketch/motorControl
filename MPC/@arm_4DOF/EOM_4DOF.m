% function EOM_4DOF

%%% 4 DOF EOM
syms th1 th2 th3 th4 l1 l2 l3 l4 l5 shum srad mhum mrad I1 I2 I3 I4 'real'


%%% Helpful Anonymous functions
% Rotation matrices (in radians)
Rx = @(x) [ 1, 0, 0; 0, cos(x), -sin(x); 0, sin(x), cos(x) ];
Ry = @(x) [ cos(x), 0, sin(x); 0, 1, 0; -sin(x), 0, cos(x) ];
Rz = @(x) [ cos(x), -sin(x), 0; sin(x), cos(x), 0; 0, 0, 1 ];

% Location of each joint
T01 = [ Ry(th1), [ 0; 0; l1 ]; zeros(1,3), 1 ]; % Should abduction
T12 = [ Rx(th2), [ 0; 0; l2 ]; zeros(1,3), 1 ]; % Shoulder extension
T23 = [ Rz(th3), [ 0; 0; l3 ]; zeros(1,3), 1 ]; % Shoulder rotation
T34 = [ Rx(th4), [ 0; 0; l4 ]; zeros(1,3), 1 ]; % Elbow flexion
T45 = [ zeros(3,3), [ 0; 0; l5 ]; zeros(1,3), 1 ]; % Hand position

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;

%% Find joint space equations of motion
% Find the location of the center of mass of each link
T0hum = T01 * T12 * [ zeros(3,3), [ 0; 0; srad ]; zeros(1,3), 1 ];
T0rad = T01 * T12 * T23 * T34 * [ zeros(3,3), [ 0; 0; shum ]; zeros(1,3), 1 ];

Jvhum = [ diff(T0hum(1:3,4),'th1'), diff(T0hum(1:3,4),'th2'), ...
    diff(T0hum(1:3,4),'th3'), diff(T0hum(1:3,4),'th4') ]; 
Jvrad = [ diff(T0rad(1:3,4),'th1'), diff(T0rad(1:3,4),'th2'), ...
    diff(T0rad(1:3,4),'th3'), diff(T0rad(1:3,4),'th4') ]; 
Jw1 = [ diff(T01(1:3,4),'th1'), diff(T01(1:3,4),'th2'), ...
    diff(T01(1:3,4),'th3'), diff(T01(1:3,4),'th4') ]; 
Jw2 = [ diff(T02(1:3,4),'th1'), diff(T02(1:3,4),'th2'), ...
    diff(T02(1:3,4),'th3'), diff(T02(1:3,4),'th4') ]; 
Jw3 = [ diff(T03(1:3,4),'th1'), diff(T03(1:3,4),'th2'), ...
    diff(T03(1:3,4),'th3'), diff(T03(1:3,4),'th4') ]; 
Jw4 = [ diff(T04(1:3,4),'th1'), diff(T04(1:3,4),'th2'), ...
    diff(T04(1:3,4),'th3'), diff(T04(1:3,4),'th4') ]; 

% Mass matrix, A
A = simplify( mhum*Jvhum'*Jvhum + mrad*Jvrad'*Jvrad + I1*Jw1'*Jw1 + ...
    I2*Jw2'*Jw2 + I3*Jw3'*Jw3 + I4*Jw4'*Jw4 );

% Centrifugal and Coriolis
chris = @(i,j,k) simplify( 1/2*( diff(A(i,j),k) + diff(A(i,k),j) - ...
    diff(A(j,k),i))); % Christoffel symbols

B = 2.* [ chris(1,1,2); chris(2,1,2)

% Gravity effects
g = [ 0, 0, -9.81 ]';
<<<<<<< HEAD:MPC/@arm_4DOF/EOM_4DOF.m
g = -( Jvhum'*mhum*g + Jvrad'*mrad*g );


%% Find Operational space equations of motion
=======
G = -( Jvhum'*mhum*g + Jvrad'*mrad*g );


% Apply subject specific parameters
% th1 = 0; 
% th2 = 0; 
% th3 = pi/2;
% th4 = pi/2;
% l1 = 0;
% l2 = 0; 
% l3 = 0; 
% l4 = 1; 
% l5 = 1;
% srad = 0.5;
% shum = 0.5;
>>>>>>> origin/master:MPC/EOM_4DOF.m
