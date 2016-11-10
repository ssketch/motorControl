% function EOM_4DOF
clear; clc; close all;

%%% 4 DOF EOM
syms th1 th2 th3 th4 l1 l2 l3 l4 l5 shum srad mhum mrad I1 I2 I3 I4 'real'
l1 = 0;
l2 = 0;

%%% Helpful Anonymous functions
% Rotation matrices (in radians)
Rx = @(x) [ 1, 0, 0; 0, cos(x), -sin(x); 0, sin(x), cos(x) ];
Ry = @(x) [ cos(x), 0, sin(x); 0, 1, 0; -sin(x), 0, cos(x) ];
Rz = @(x) [ cos(x), -sin(x), 0; sin(x), cos(x), 0; 0, 0, 1 ];

% Location of each joint
T01 = [ Ry(th1), [ 0; 0; 0 ]; zeros(1,3), 1 ]; % Should abduction
T12 = [ Rx(th2), [ 0; l1; 0 ]; zeros(1,3), 1 ]; % Shoulder extension
T23 = [ Rz(th3), [ 0; 0; -l2 ]; zeros(1,3), 1 ]; % Shoulder rotation
T34 = [ Ry(th4), [ 0; 0; -l3 ]; zeros(1,3), 1 ]; % Elbow flexion
T45 = [ eye(3), [ 0; 0; -l4 ]; zeros(1,3), 1 ]; % Hand position

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;

%% Find joint space equations of motion
% Find the location of the center of mass of each link
T0hum = T03 * [ eye(3), [ 0; 0; -shum ]; zeros(1,3), 1 ];
T0rad = T04 * [ eye(3), [ 0; 0; -srad ]; zeros(1,3), 1 ];

% Compute the Jacobian.  This includes the end-point Jacobian as well as
% intermediate Jacobians.
Jvhum = [ diff(T0hum(1:3,4),'th1'), diff(T0hum(1:3,4),'th2'), ...
    diff(T0hum(1:3,4),'th3'), diff(T0hum(1:3,4),'th4') ]; 
Jvrad = [ diff(T0rad(1:3,4),'th1'), diff(T0rad(1:3,4),'th2'), ...
    diff(T0rad(1:3,4),'th3'), diff(T0rad(1:3,4),'th4') ]; 

Jvhum = simplify( jacobian( T0hum(1:3,4), [ th1 th2 th3 th4 ] ));
Jvrad = simplify( jacobian( T0rad(1:3,4), [ th1 th2 th3 th4 ] ));

Jwhum = [ T01(1:3,3), T12(1:3,3), T23(1:3,3), zeros(3,1) ];
Jwrad = [ T01(1:3,3), T12(1:3,3), T23(1:3,3), T34(1:3,3) ];

Jv1 = [ diff(T01(1:3,4),'th1'), diff(T01(1:3,4),'th2'), ...
    diff(T01(1:3,4),'th3'), diff(T01(1:3,4),'th4') ]; 
Jv2 = [ diff(T02(1:3,4),'th1'), diff(T02(1:3,4),'th2'), ...
    diff(T02(1:3,4),'th3'), diff(T02(1:3,4),'th4') ]; 
Jv3 = [ diff(T03(1:3,4),'th1'), diff(T03(1:3,4),'th2'), ...
    diff(T03(1:3,4),'th3'), diff(T03(1:3,4),'th4') ]; 
Jv4 = [ diff(T04(1:3,4),'th1'), diff(T04(1:3,4),'th2'), ...
    diff(T04(1:3,4),'th3'), diff(T04(1:3,4),'th4') ]; 
Jv5 = simplify( jacobian( T05(1:3,4), [th1 th2 th3 th4]));

% Jw1 = 

% Mass matrix, A
A = simplify( mhum*Jvhum'*Jvhum + Jwhum'*I3*Jwhum + mrad*Jvrad'*Jvrad + ...
    Jwrad'*I4*Jwrad);

% Centrifugal and Coriolis
chris = @(i,j,k) simplify( 1/2*( diff(A(i,j),k) + diff(A(i,k),j) - ...
    diff(A(j,k),i))); % Christoffel symbols

% Coriolis
B = 2.* [ chris(1,1,2), chris(1,1,3), chris(1,1,4), chris(1,2,3), chris(1,2,4), chris(1,3,4);
          chris(2,1,2), chris(2,1,3), chris(2,1,4), chris(2,2,3), chris(2,2,4), chris(2,3,4);
          chris(3,1,2), chris(3,1,3), chris(3,1,4), chris(3,2,3), chris(3,2,4), chris(3,3,4);
          chris(4,2,3), chris(4,1,3), chris(4,1,4), chris(4,2,3), chris(4,2,4), chris(4,3,4) ];
% Centripetal
C = [ chris(1,1,1), chris(1,2,2), chris(1,3,3), chris(1,4,4);
      chris(2,1,1), chris(2,2,2), chris(2,3,3), chris(2,4,4);
      chris(3,1,1), chris(3,2,2), chris(3,3,3), chris(3,4,4);
      chris(4,1,1), chris(4,2,2), chris(4,3,3), chris(4,4,4) ];
      
% Gravity effects
g = [ 0, 0, -9.81 ]';
g = -( Jvhum'*mhum*g + Jvrad'*mrad*g );


%% Compute the time derivative of the Jacobian
syms t
th1 = symfun( sym('th1(t)'), t);
th2 = symfun( sym('th2(t)'), sym('t'));
th3 = symfun( sym('th3(t)'), sym('t'));
th4 = symfun( sym('th4(t)'), sym('t'));

J = eval(Jv5)
J_dot = diff( J, t )

%% Find Operational space equations of motion
G = -( Jvhum'*mhum*g + Jvrad'*mrad*g );

% 
% % Apply subject specific parameters
% th1 = 0; 
% th2 = 0; 
% th3 = 0;
% th4 = 0;
% % l1 = 0;
% % l2 = 0; 
% % l4 = 1; 
% % l5 = 1;
% % srad = 0.5;
% % shum = 0.5;
% I1 = 0;
% I2 = 0;
% I4 = 0;
