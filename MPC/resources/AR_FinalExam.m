%%% This is my code from my final exam for Advanced Robotics.  It's got a
%%% lot of stuff already written to compute the equations of motion for
%%% some complicated systems.  I'm using it to get the 4DOF model dynamics.
tic; clc; clear; close all; format compact;
set(0,'DefaultFigureWindowStyle','docked');

%%% Helpful Anonymous functions
% Rotation matrices
Rx = @(x) [ 1, 0, 0; 0, cos(x), -sin(x); 0, sin(x), cos(x) ];
Ry = @(x) [ cos(x), 0, sin(x); 0, 1, 0; -sin(x), 0, cos(x) ];
Rz = @(x) [ cos(x), -sin(x), 0; sin(x), cos(x), 0; 0, 0, 1 ];
% Convert DH parameters to transformation matrix
DH2T = @(a, alpha, d, theta) [ Rx(alpha), [a;0;0]; zeros(1,3), 1]*...
    [ Rz(theta), [0;0;d]; zeros(1,3), 1];

%% Problem 1
syms th1 th2 d3 L1 m1 m2 m3 I1 'real'

% Part b: Find the Jacobian Matrix associated with these operational
% coordinates
T01 = DH2T( 0, 0, 0, th1 );
T12 = DH2T( L1, 0, 0, th2 );
T23 = DH2T( d3, 0, 0, 0 );

T02 = simplify( T01 * T12 );
T03 = simplify( T01 * T12 * T23 );

xp = [ T03(1:2,4); th1+th2 ];

J0 = simplify( eval([ diff(xp,'th1'), diff(xp,'th2'), diff(xp,'d3') ]));


% Part c: Find the Jacobian matrix in frame {2}.
J2 = simplify( T02(1:3,1:3)' * J0 );


% Part d: Find the joint spcae kinetic energy matrix A(q) and the joint
% space gravity vector G(q).
Jv1 = simplify([ diff(T01(1:3,4),'th1'), diff(T01(1:3,4),'th2'), ...
    diff(T01(1:3,4),'d3') ]); 
Jv2 = simplify([ diff(T02(1:3,4),'th1'), diff(T02(1:3,4),'th2'), ...
    diff(T02(1:3,4),'d3') ]); 
Jv3 = simplify([ diff(T03(1:3,4),'th1'), diff(T03(1:3,4),'th2'), ...
    diff(T03(1:3,4),'d3') ]); 
Jw1 = simplify([ T01(1:3,3), zeros(3,2) ]);

A = simplify( m1*Jv1'*Jv1 + m2*Jv2'*Jv2 + m3*Jv3'*Jv3 + I1 * Jw1' * Jw1 );

g = [ 0, -9.81, 0 ]';
G = -( Jv1'*m1*g + Jv2'*m2*g + Jv3'*m3*g );


% Part e: Find the inverse of the operational space kinetic energy matrix
% in frame {2}.
L2 = simplify( inv(J2' \ A * inv(J2)) );
L2_inv = simplify( inv( L2 ));


% Part f: Determine the end-effector acceleration
th1 = 0;
th2 = 45 * pi/180;
d3 = 1;
L1 = 1;
I1 = 1;
m2 = 1;
m3 = 1;

x_ddot = eval( L2_inv * T02(1:3,1:3)' * [ 0, 10, 0 ]' );







%% Problem 2
% Part a: Determine the generalized selection matrices
sigma = [ 0, 0, 0; 0, 1, 0; 0, 0, 1 ];

sigma_bar = eye(3) - sigma;

R30 = T03(1:3,1:3)';

Omega = R30 * sigma * R30';
Omega_bar = R30 * sigma_bar * R30';


% Part b: Find the inverse of lambda_v in frame {2}
L2_inv_v = L2_inv(1:2,1:2);


% Part c: What is the effective mass in the X_0, Y_0, X_2, and Y_2
% directions?
peanut_v = @(x,y) 1./([x;y]'*L2_inv_v*[x;y]);

% Frame 2:
x_2 = peanut_v(1,0);
y_2 = peanut_v(0,1);

% Frame 0:
x_0 = T02(1:3,1:3)'*[1,0,0]';
y_0 = T02(1:3,1:3)'*[0,1,0]';

x_0 = peanut_v(x_0(1), x_0(2))
y_0 = peanut_v(y_0(1), y_0(2))

%%% Plot the peanut
    figure('name', '2.c')
    x_0 = [];
    y_0 = [];
    x_2 = [];
    y_2 = [];
    m_0 = [];
    m_2 = [];
    for th = linspace( 0, 2*pi, 100 )
        u = Rz(th)*[1;0;0];
        x_0 = [ x_0 u(1)];
        y_0 = [ y_0 u(2)];
        z = T02(1:3,1:3)'*u;
        x_2 = [ x_2 z(1) ];
        y_2 = [ y_2 z(2) ];
        m = peanut_v( x_0(end), y_0(end) );
        m_0 = [m_0 m ];
        m = peanut_v( x_2(end), y_2(end) );
        m_2 = [m_2 m ];
    end

    d3 = 1;
    L1 = 1;
    I1 = 1;
    m2 = 1;
    m3 = 1;
    th2 = pi/5; 
    th1 =0 ;
    m_0 = eval( m_0);
    m_2 = eval( m_2);
    x_2 = eval( x_2);
    y_2 = eval( y_2 );

    plot( x_0.*m_0, y_0.*m_0, x_0.*m_2, y_0.*m_2, 'LineWidth', 2 )
        axis equal
        xlabel 'X_0 Direction'
        ylabel 'Z_0 Direction'
        legend( 'Problem 1', 'Problem 2', 'Location', 'SouthEast' )
        box off
        legend('boxoff')



% Part e: Find the inverse of lambda_w in frame {2}
L2_inv_w = L2_inv(3,3);
% J2_w = J2(:,3);
% L2_inv_w = pinv(J2_w) * A * pinv(J2_w)';

% Part f: What is the effective inertia about the z_0 axis?
peanut_w = @(alpha) 1./(alpha'*L2_inv_w*alpha);

I_z0 = peanut_w( 1 );

%%% Plot the peanut:
    figure('name', '2.f')
    I = [];
    x = [];
    y = [];
    for th = linspace( 0, 2*pi, 100 )
        u = Rz(th)*[1;0;0];
        x = [x u(1)];
        y = [y u(2)];
        Ii = peanut_w( th );
        I = [I Ii ];
    end

    d3 = 1;
    L1 = 1;
    I1 = 1;
    m2 = 1;
    m3 = 1;
    I = eval( I );
    plot( x.*I, y.*I, 'LineWidth', 2 )
        axis equal
        xlabel 'X_0 Direction'
        ylabel 'Y_0 Direction'
        box off


% Part g: find the configurations at which the effective inertia about the
% z_0 axis is maximum and minimum
maxI = 0;
minI = inf;

for th1 = linspace( 0, 2*pi, 100 )
    for th2 = linspace( 0, 2*pi, 100 )

        I = eval( peanut_w( 1 ));

        if I > maxI
            maxI = I;
            th1max = th1;
            th2max = th2;
        end
        if I < minI
            minI = I;
            th1min = th1;
            th2min = th2;
        end
    end
end






%% Problem 3
% Part a: Select a set of operational space coordinates for this task
% x, y

% Part b: Find the Jacobian in frame {2} associated with the above task.
% J0_v = J0(1:2,:);   % omits the column corresponding to alpha
% J2_v = simplify( T02(1:3,1:3)' * [ J0_v );    % convert to frame {2}

xp = [ T03(1:2,4) ];
J0_v = ( ([ diff(xp,'th1'), diff(xp,'th2'), diff(xp,'d3') ])); % frame 0
J2_v = simplify( T02(1:3,1:3)' * [J0_v; zeros(1,3)] ); % frame 2
J2_v = J2_v(1:2,:);

% Part c: Find the operational space kinetic energy matrix in frame 2
L2_v = inv( L2_inv_v );


% Part d: Find the dynamically consistent Jacobian inverse in frame 2.
J2_inv_v = A \ J2_v' * L2_v; 


% Part e: find the operational space gravity forces in frame {2}.  Find the
% corresponding joint space forces/torques.
P2 = J2' \ G;
% The torques/forces required to compensate for gravity are given by the
% joint space gravity forces/torques vector, G(q).  That is, ?=G(q).


% Part f: Find the matrix that maps the joint forces/torques into a
% dynamically consistent null space.  Use this matrix to determine the
% gravity forces/torques acting in the null space.
Np = eye(3) - J2_inv_v*J2_v; 

J2p = simplify( J2*Np );
% Lambda2p = simplify( inv(J2p*inv(A)*J2p'));
% J2p_bar = inv(A)*J2p'*Lambda2p;
% p2p = J2p_bar'*g;
G2p = J2p*G;



%% Problem 4
syms q1 q2 q3 L 'real'
% Part a: Compute the Jacobian with respect to your operational coordinates
% and frame {0}.
T01 = DH2T( 0, 0, 0, pi/2 + q1 );
T12 = DH2T( q2, pi/2, 0, 0 );
T23 = DH2T( 0, 0, 0, q3 );
T34 = DH2T( L, 0, 0, 0 );

T02 = simplify( T01 * T12 );
T03 = simplify( T01 * T12 * T23 );
T04 = simplify( T01 * T12 * T23 * T34 );

xp = [ T04(1:3,4) ];

J = simplify( ([ diff(xp,'q1'), diff(xp,'q2'), diff(xp,'q3') ]));


% Part b: What is the effective mass in the x4 and y4 directions?
m1 = 1;
m3 = 1;
m4 = 1;
L = 1;
I = 1;
q1 = 0;
q2 = 0.3;
q3 = 120 * pi/ 180;

Jv1 = simplify([ diff(T01(1:3,4),'q1'), diff(T01(1:3,4),'q2'), ...
    diff(T01(1:3,4),'q3') ]); 
Jv3 = simplify([ diff(T03(1:3,4),'q1'), diff(T03(1:3,4),'q2'), ...
    diff(T03(1:3,4),'q3') ]); 
Jv4 = simplify([ diff(T04(1:3,4),'q1'), diff(T04(1:3,4),'q2'), ...
    diff(T04(1:3,4),'q3') ]); 
Jw1 = simplify([ T01(1:3,3), zeros(3,2) ]);
Jw3 = simplify([ Jw1(:,1) T02(1:3,3) T03(1:3,3) ]);

A = m1*Jv1'*Jv1 + m3*Jv3'*Jv3 + m4*Jv4'*Jv4 + I*Jw1'*Jw1 + I*Jw3'*Jw3;
A = eval( A );

L_inv =  inv( J*inv(A)*J' );
L_inv = eval( L_inv );

peanut_4 = @(x,y,z) 1./([x;y;z]'*L_inv*[x;y;z]);

% Effective mass in:
x4 = peanut_4(1, 0, 0);
y4 = peanut_4(0, 1, 0);
% The effective masses in the x_4 and the y_4 directions are different.
% This is because, in this configuration, joint 2 supports more load in one
% direction than the other.  Meanwhile, joint 1 does not contribute in any
% way (it is rigid in both the x_4 and y_4 directions).



%%% Plot the peanut
    figure('name', '4.b')
    x = [];
    y = [];
    m = [];
    for th = linspace( 0, 2*pi, 200 )
        u = Rz(th)*[1;0;0];
        
        x = [ x u(1) ];
        y = [ y u(2) ];
        
        mi = peanut_4( x(end), y(end), 0 );
        m = [m mi ];

    end
    
    plot( x.*m, y.*m, 'LineWidth', 2 )
        axis equal
        xlabel 'x_4 Direction'
        ylabel 'y_4 Direction'
        box off


% Part c: What is the effective mass in the z0 direction for this
% configuration?
z0 = [];
for th = linspace( 120, 179 ) * pi/180
    q3 = th;
    
    A = m1*Jv1'*Jv1 + m3*Jv3'*Jv3 + m4*Jv4'*Jv4 + I*Jw1'*Jw1 + I*Jw3'*Jw3;
    A = eval( A );

    L_inv =  inv( J*inv(A)*J' );
    L_inv = eval( L_inv );

    peanut_4 = @(x,y,z) 1./([x;y;z]'*L_inv*[x;y;z]);
    
    u = eval(T04(1:3,1:3)')*[0;0;1]; % Change z_0 to frame {4}
    
    zi = peanut_4(u(1), u(2), u(3));
    z0 = [z0, zi ];
end

figure( 'name', '4.c' )
    plot( linspace( 120, 197 )*pi/180, z0 )
    hold on
    text( 2.15, 0.0246, [num2str(z0(1)) ' at q_3 = 120?' ]);
    plot( 120*pi/180, z0(1), 'g*' )
% The effective mass in the z_0 increases as q_3 approaches 180? because
% only joint 3 supports loads applied at the end-effector in the z_0
% direction and the moment arm between joint 3 and the end-effector
% increases as q_3 goes from 120 to 180?.  This means that the required
% torque from q_3 to support against load applied in this direction also
% increases.








%% Problem 5
J1 = [ -3*sqrt(3), -sqrt(3), -sqrt(3); 3, 3, 3 ];
J2 = [ -3*sqrt(3), -2*sqrt(3), -sqrt(3); -3, -6, -3 ];
W = 12; % N

% Part a: what is the vector of torques that would be needed to carry this
% load by Arm 1 alone?
T1 = J1'*[0;-W];
% T1 = [ -36  -36  -36 ]'

% Part b: what is the vector of torques that would be needed to carry this
% load by Arm 2 alone?
T2 = J2'*[0;-W];
% T2 = [ 36  72  36 ]'

% Part c: Find the coefficient, alpha, that shares the load between the two
% arms with equal actuator efforts
B = @(T) max( abs( T / 52 ));

alpha = linspace( 0, 1 );
figure('name', '5.c' )
plot( alpha, B( J1'*[0;-W]*alpha), alpha,  B( J2'*[0;-W]*(1-alpha)) )    
% The two lines produced by this plot intersect at alpha = 0.6667


% Part d: What are the corresponding torque vectors for Arm 1 and Arm 2,
% compare with part a and b
alpha = 0.6667;
T1 = J1'*[0;-W]*alpha;
% T1 = [ -24.0012  -24.0012  -24.0012 ]'
T2 = J2'*[0;-W]*(1-alpha);
% T2 =[ 11.9988  23.9976  11.9988 ]'

% Both arms require less effort using the load sharing procedure according
% to the definition of actuator effort provided.  Additionally, while
% before sharing the load, Arm 2 exceeded its maximum allowed torque
% constraint and does not when sharing the load equally with Arm 1.
