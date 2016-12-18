% function EOM_4DOF
% This script computes the equations of motion of the 4 degree-of-freedom
% arm model.
clear; clc; close all;

%%% 4 DOF EOM
syms th1 th2 th3 th4 l1 l2 s1 s2 m1 m2 I1 I2 'real'

%%% Helpful Anonymous functions
% Rotation matrices
Rx = @(x) [ 1, 0, 0; 0, cos(x), -sin(x); 0, sin(x), cos(x) ];
Ry = @(x) [ cos(x), 0, sin(x); 0, 1, 0; -sin(x), 0, cos(x) ];
Rz = @(x) [ cos(x), -sin(x), 0; sin(x), cos(x), 0; 0, 0, 1 ];
% Convert DH parameters to transformation matrix
DH2T = @(a, alpha, d, theta) [ Rx(alpha), [a;0;0]; zeros(1,3), 1]*...
    [ Rz(theta), [0;0;d]; zeros(1,3), 1];
% Christoffel symbol anonymous function defined later because it depends on
% the mass matrix, A
% Christoffel symbols
chris = @(A,i,j,k) simplify( 1/2*( diff(A(i,j),k) + diff(A(i,k),j) - ...
    diff(A(j,k),i)));


%% Forward kinematics
digits(3);  % Only care about 3 digits of precision

T01 = DH2T( 0,  pi/2,  0,  pi/2 + th1 ); % Shoulder extension
T12 = DH2T( 0,  pi/2,  0,         th2 ); % Shoulder abduction
T23 = DH2T( 0, -pi/2, l1, -pi/2 + th3 ); % Shoulder rotation
T34 = DH2T( 0, -pi/2,  0,         th4 ); % Elbow flexion
T45 = DH2T( 0,  pi/2, l2,           0 ); % Hand position

% syms x y
% a = {your expression}
% [c,t] = coeffs(a,'{x,y}');
% kr = find(abs(double(real(c))) > 1.e-8);
% ki = find(abs(double(imag(c))) > 1.e-8);
% cr = vpa(real(c(kr)),8);
% ci = vpa(imag(c(ki)),8);
% b = sum([cr.*t(kr) ci.*t(ki)])

for i = 1:length(T01(:,1))
    for j = 1:length(T01(1,:))
        a = T01(i,j);
        [c,t] = coeffs(a);
        kr = find(abs(double(real(c))) > 1.e-8);
        ki = find(abs(double(imag(c))) > 1.e-8);
        cr = vpa(real(c(kr)),8);
        ci = vpa(imag(c(ki)),8);
        T01(i,j) = sum([cr.*t(kr) ci.*t(ki)]);
        
        a = T12(i,j);
        [c,t] = coeffs(a);
        kr = find(abs(double(real(c))) > 1.e-8);
        ki = find(abs(double(imag(c))) > 1.e-8);
        cr = vpa(real(c(kr)),8);
        ci = vpa(imag(c(ki)),8);
        T12(i,j) = sum([cr.*t(kr) ci.*t(ki)]);
        
        a = T23(i,j);
        [c,t] = coeffs(a);
        kr = find(abs(double(real(c))) > 1.e-8);
        ki = find(abs(double(imag(c))) > 1.e-8);
        cr = vpa(real(c(kr)),8);
        ci = vpa(imag(c(ki)),8);
        T23(i,j) = sum([cr.*t(kr) ci.*t(ki)]);
        
        a = T34(i,j);
        [c,t] = coeffs(a);
        kr = find(abs(double(real(c))) > 1.e-8);
        ki = find(abs(double(imag(c))) > 1.e-8);
        cr = vpa(real(c(kr)),8);
        ci = vpa(imag(c(ki)),8);
        T34(i,j) = sum([cr.*t(kr) ci.*t(ki)]);
        
        a = T45(i,j);
        [c,t] = coeffs(a);
        kr = find(abs(double(real(c))) > 1.e-8);
        ki = find(abs(double(imag(c))) > 1.e-8);
        cr = vpa(real(c(kr)),8);
        ci = vpa(imag(c(ki)),8);
        T45(i,j) = sum([cr.*t(kr) ci.*t(ki)]);
    end
end

T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;

%% Find joint space equations of motion
% Find the location of the center of mass of each link
T0COM1 = vpa( T02 * [ eye(3), [ 0;  s1; 0 ]; zeros(1,3), 1 ]);
T0COM2 = vpa( T04 * [ eye(3), [ 0; -s2; 0 ]; zeros(1,3), 1 ]);

% Compute the Jacobian.  This includes the end-point Jacobian as well as
% intermediate Jacobians.
Jv1 = vpa( simplify( jacobian( T0COM1(1:3,4), [ th1 th2 th3 th4 ] )));
Jv2 = vpa( simplify( jacobian( T0COM2(1:3,4), [ th1 th2 th3 th4 ] )));

Jw1 = [ T01(1:3,3), T12(1:3,3), T23(1:3,3), zeros(3,1) ];
Jw2 = [ T01(1:3,3), T12(1:3,3), T23(1:3,3), T34(1:3,3) ];

% Just in case we want to put anything in the hand
Jv5 = simplify( jacobian( T05(1:3,4), [th1 th2 th3 th4]));


% Mass matrix, A
A = simplify( m1*Jv1'*Jv1 + Jw1'*I1*Jw1 + m2*Jv2'*Jv2 + ...
    Jw2'*I2*Jw2);

% Centrifugal and Coriolis

% Coriolis
B = 2.* [ chris(A,1,1,2), chris(A,1,1,3), chris(A,1,1,4), chris(A,1,2,3), chris(A,1,2,4), chris(A,1,3,4);
          chris(A,2,1,2), chris(A,2,1,3), chris(A,2,1,4), chris(A,2,2,3), chris(A,2,2,4), chris(A,2,3,4);
          chris(A,3,1,2), chris(A,3,1,3), chris(A,3,1,4), chris(A,3,2,3), chris(A,3,2,4), chris(A,3,3,4);
          chris(A,4,2,3), chris(A,4,1,3), chris(A,4,1,4), chris(A,4,2,3), chris(A,4,2,4), chris(A,4,3,4) ];
% Centripetal
C = [ chris(A,1,1,1), chris(A,1,2,2), chris(A,1,3,3), chris(A,1,4,4);
      chris(A,2,1,1), chris(A,2,2,2), chris(A,2,3,3), chris(A,2,4,4);
      chris(A,3,1,1), chris(A,3,2,2), chris(A,3,3,3), chris(A,3,4,4);
      chris(A,4,1,1), chris(A,4,2,2), chris(A,4,3,3), chris(A,4,4,4) ];
      
% Gravity effects
g = [ 0, 0, -9.81 ]';
g = -( Jv1'*m1*g + Jv2'*m2*g );


%% Compute the time derivative of the Jacobian
syms t
th1 = symfun( sym('th1(t)'), t);
th2 = symfun( sym('th2(t)'), sym('t'));
th3 = symfun( sym('th3(t)'), sym('t'));
th4 = symfun( sym('th4(t)'), sym('t'));

J = eval(Jv5);
J_dot = vpa( diff( J, t ), 3 );

%% Find Operational space equations of motion
% G = -( Jv1'*mhum*g + Jv2'*mrad*g );

