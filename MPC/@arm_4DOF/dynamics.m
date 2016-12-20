% This function returns the exuation of motion for the arm in its current
% state, represented in either joint or task space. In joint space, it is
% of the form x_dot = f(x,u) where u is the vector of joint torxues. In
% task space, it is of the form x_dot = f(x,u) where u is the vector of
% hand forces.
function f = dynamics(arm, x, u)

% if no input is specified, use stored value
if nargin < 3
    u = arm.u.val;
    
    % if no state is specified, use current arm state
    if nargin < 2
        x = arm.x.val;
    end
end

% Note: results are copy-pasted in from the EOM_4DOF.m script provided in
% the resources folder and then the control-f function used to replace
% parameters with the correct syntax here; terms less than 1e-3 due to
% numerical errors were rounded down to 0 using the same method

% Mass matrix
M11 = arm.I1 + arm.I2 + arm.m2*(arm.s2*(sin(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) - sin(x(1) + 1.57)*cos(x(4))*sin(x(2))) - arm.l1*sin(x(1) + 1.57)*sin(x(2)))^2 + arm.m2*(arm.s2*(sin(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + cos(x(1) + 1.57)*cos(x(4))*sin(x(2))) + arm.l1*cos(x(1) + 1.57)*sin(x(2)))^2 + arm.m1*arm.s1^2*cos(x(1) + 1.57)^2*sin(x(2))^2 + arm.m1*arm.s1^2*sin(x(1) + 1.57)^2*sin(x(2))^2;
M12 = arm.I1 + arm.I2 + arm.m2*(arm.s2*(cos(x(1) + 1.57)*cos(x(2))*cos(x(4)) + cos(x(1) + 1.57)*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) + arm.l1*cos(x(1) + 1.57)*cos(x(2)))*(arm.s2*(sin(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) - sin(x(1) + 1.57)*cos(x(4))*sin(x(2))) - arm.l1*sin(x(1) + 1.57)*sin(x(2))) + arm.m2*(arm.s2*(sin(x(1) + 1.57)*cos(x(2))*cos(x(4)) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) + arm.l1*sin(x(1) + 1.57)*cos(x(2)))*(arm.s2*(sin(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + cos(x(1) + 1.57)*cos(x(4))*sin(x(2))) + arm.l1*cos(x(1) + 1.57)*sin(x(2)));
M13 = arm.m2*arm.s2*sin(x(4))*(arm.s2*(sin(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) - sin(x(1) + 1.57)*cos(x(4))*sin(x(2))) - arm.l1*sin(x(1) + 1.57)*sin(x(2)))*(sin(x(1) + 1.57)*cos(x(3) - 1.57) + cos(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57)) - arm.I2 - arm.m2*arm.s2*sin(x(4))*(arm.s2*(sin(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + cos(x(1) + 1.57)*cos(x(4))*sin(x(2))) + arm.l1*cos(x(1) + 1.57)*sin(x(2)))*(cos(x(1) + 1.57)*cos(x(3) - 1.57) - sin(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57)) - arm.I1;
M14 = arm.m2*arm.s2*(arm.s2*(sin(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) - sin(x(1) + 1.57)*cos(x(4))*sin(x(2))) - arm.l1*sin(x(1) + 1.57)*sin(x(2)))*(cos(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) - cos(x(1) + 1.57)*sin(x(2))*sin(x(4))) - arm.m2*arm.s2*(cos(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + sin(x(1) + 1.57)*sin(x(2))*sin(x(4)))*(arm.s2*(sin(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + cos(x(1) + 1.57)*cos(x(4))*sin(x(2))) + arm.l1*cos(x(1) + 1.57)*sin(x(2))) - arm.I2;

M22 = arm.I1 + arm.I2 + arm.m2*(arm.s2*(cos(x(4))*sin(x(2)) - cos(x(3) - 1.57)*cos(x(2))*sin(x(4))) + arm.l1*sin(x(2)))^2 + arm.m2*(arm.s2*(cos(x(1) + 1.57)*cos(x(2))*cos(x(4)) + cos(x(1) + 1.57)*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) + arm.l1*cos(x(1) + 1.57)*cos(x(2)))^2 + arm.m2*(arm.s2*(sin(x(1) + 1.57)*cos(x(2))*cos(x(4)) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) + arm.l1*sin(x(1) + 1.57)*cos(x(2)))^2 + arm.m1*arm.s1^2*sin(x(2))^2 + arm.m1*arm.s1^2*cos(x(1) + 1.57)^2*cos(x(2))^2 + arm.m1*arm.s1^2*sin(x(1) + 1.57)^2*cos(x(2))^2;
M23 = arm.m2*arm.s2*sin(x(4))*(arm.s2*(cos(x(1) + 1.57)*cos(x(2))*cos(x(4)) + cos(x(1) + 1.57)*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) + arm.l1*cos(x(1) + 1.57)*cos(x(2)))*(sin(x(1) + 1.57)*cos(x(3) - 1.57) + cos(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57)) - arm.I2 - arm.m2*arm.s2*sin(x(4))*(arm.s2*(sin(x(1) + 1.57)*cos(x(2))*cos(x(4)) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) + arm.l1*sin(x(1) + 1.57)*cos(x(2)))*(cos(x(1) + 1.57)*cos(x(3) - 1.57) - sin(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57)) - arm.I1 + arm.m2*arm.s2*sin(x(3) - 1.57)*sin(x(2))*sin(x(4))*(arm.s2*(cos(x(4))*sin(x(2)) - cos(x(3) - 1.57)*cos(x(2))*sin(x(4))) + arm.l1*sin(x(2)));
M24 = arm.m2*arm.s2*(arm.s2*(cos(x(1) + 1.57)*cos(x(2))*cos(x(4)) + cos(x(1) + 1.57)*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) + arm.l1*cos(x(1) + 1.57)*cos(x(2)))*(cos(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) - cos(x(1) + 1.57)*sin(x(2))*sin(x(4))) - arm.I2 + arm.m2*arm.s2*(arm.s2*(cos(x(4))*sin(x(2)) - cos(x(3) - 1.57)*cos(x(2))*sin(x(4))) + arm.l1*sin(x(2)))*(cos(x(2))*sin(x(4)) - cos(x(3) - 1.57)*cos(x(4))*sin(x(2))) - arm.m2*arm.s2*(arm.s2*(sin(x(1) + 1.57)*cos(x(2))*cos(x(4)) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) + arm.l1*sin(x(1) + 1.57)*cos(x(2)))*(cos(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + sin(x(1) + 1.57)*sin(x(2))*sin(x(4)));

M33 = arm.I1 + arm.I2 + arm.m2*arm.s2^2*sin(x(4))^2*(sin(x(1) + 1.57)*cos(x(3) - 1.57) + cos(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57))^2 + arm.m2*arm.s2^2*sin(x(4))^2*(cos(x(1) + 1.57)*cos(x(3) - 1.57) - sin(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57))^2 + arm.m2*arm.s2^2*sin(x(3) - 1.57)^2*sin(x(2))^2*sin(x(4))^2;
M34 = arm.I2 + arm.m2*arm.s2^2*sin(x(4))*(cos(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) - cos(x(1) + 1.57)*sin(x(2))*sin(x(4)))*(sin(x(1) + 1.57)*cos(x(3) - 1.57) + cos(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57)) + arm.m2*arm.s2^2*sin(x(4))*(cos(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + sin(x(1) + 1.57)*sin(x(2))*sin(x(4)))*(cos(x(1) + 1.57)*cos(x(3) - 1.57) - sin(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57)) + arm.m2*arm.s2^2*sin(x(3) - 1.57)*sin(x(2))*sin(x(4))*(cos(x(2))*sin(x(4)) - cos(x(3) - 1.57)*cos(x(4))*sin(x(2)));

M44 = arm.I2 + arm.m2*arm.s2^2*(cos(x(2))*sin(x(4)) - cos(x(3) - 1.57)*cos(x(4))*sin(x(2)))^2 + arm.m2*arm.s2^2*(cos(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + sin(x(1) + 1.57)*sin(x(2))*sin(x(4)))^2 + arm.m2*arm.s2^2*(cos(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) - cos(x(1) + 1.57)*sin(x(2))*sin(x(4)))^2;

M21 = M12;

M31 = M13;
M32 = M23;

M41 = M14;
M42 = M24;
M43 = M34;
 
M = [ M11 M12 M13 M14;
      M21 M22 M23 M24;
      M31 M32 M33 M34;
      M41 M42 M43 M44 ];
  
  
% Coriolis
B11 = 0;
B12 = 0;
B13 = 0; 
B14 = arm.m2*arm.s2*(arm.s2*cos(x(2))*cos(x(4))^2 - arm.s2*cos(x(2)) - arm.s2*cos(x(3) - 1.57)*cos(x(2))*sin(x(3) - 1.57) + arm.l1*cos(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.l1*sin(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(4))*sin(x(2))*sin(x(4)) + arm.s2*cos(x(4))*sin(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(2))*cos(x(4))^2*sin(x(3) - 1.57));
B15 = arm.m2*arm.s2*sin(x(3) - 1.57)*(arm.s2*sin(x(2)) + arm.l1*cos(x(2))*sin(x(4)) + arm.l1*cos(x(4))*sin(x(2)) + arm.s2*cos(x(3) - 1.57)*sin(x(2)) + arm.s2*cos(x(2))*cos(x(4))*sin(x(4)) - arm.s2*cos(x(3) - 1.57)*cos(x(4))^2*sin(x(2)));
B16 = arm.m2*arm.s2*(arm.s2*cos(x(2)) + arm.s2*cos(x(2))*sin(x(3) - 1.57) - arm.s2*cos(x(2))*cos(x(4))^2 + arm.l1*cos(x(2))*cos(x(4))*sin(x(3) - 1.57) - arm.l1*cos(x(3) - 1.57)*sin(x(2))*sin(x(4)) - arm.s2*cos(x(3) - 1.57)*cos(x(4))*sin(x(2))*sin(x(4)));

B21 = (arm.m2*arm.s2^2*(sin(2*x(3) - 2*x(4) - 3.14) - 2*sin(2*x(3) - 3.14) + sin(2*x(3) + 2*x(4) - 3.14)))/4;
B22 = arm.m2*arm.s2*(arm.s2*cos(x(2)) - arm.s2*cos(x(2))*cos(x(4))^2 - arm.s2*cos(x(3) - 1.57)*cos(x(2))*sin(x(3) - 1.57) - arm.l1*cos(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.l1*sin(x(3) - 1.57)*sin(x(2))*sin(x(4)) - arm.s2*cos(x(3) - 1.57)*cos(x(4))*sin(x(2))*sin(x(4)) + arm.s2*cos(x(4))*sin(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(2))*cos(x(4))^2*sin(x(3) - 1.57));
B23 = -arm.m2*arm.s2*sin(x(3) - 1.57)*(arm.s2*sin(x(2)) - arm.l1*cos(x(2))*sin(x(4)) + arm.l1*cos(x(4))*sin(x(2)) - arm.s2*cos(x(3) - 1.57)*sin(x(2)) - arm.s2*cos(x(2))*cos(x(4))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(4))^2*sin(x(2)));
B24 = -arm.m2*arm.s2^2*(sin(2*x(3) - 2*x(4) - 3.14) - 2*sin(2*x(3) - 3.14) + sin(2*x(3) + 2*x(4) - 3.14));
B25 = -2*arm.m2*arm.s2^2*(cos(2*x(3) - 2*x(4) - 3.14) - 2*cos(2*x(3) - 3.14) + cos(2*x(3) + 2*x(4) - 3.14));
B26 = 0;

B31 = -arm.m2*arm.s2*(arm.s2*cos(x(2)) - arm.s2*cos(x(2))*cos(x(4))^2 - arm.s2*cos(x(3) - 1.57)*cos(x(2))*sin(x(3) - 1.57) - arm.l1*cos(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.l1*sin(x(3) - 1.57)*sin(x(2))*sin(x(4)) - arm.s2*cos(x(3) - 1.57)*cos(x(4))*sin(x(2))*sin(x(4)) + arm.s2*cos(x(4))*sin(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(2))*cos(x(4))^2*sin(x(3) - 1.57));
B32 = arm.m2*arm.s2^2*sin(2*x(4));
B33 = -arm.m2*arm.s2*(arm.s2*cos(x(2))*sin(x(3) - 1.57) - arm.s2*cos(x(2)) + arm.s2*cos(x(2))*cos(x(4))^2 + arm.l1*cos(x(2))*cos(x(4))*sin(x(3) - 1.57) + arm.l1*cos(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(4))*sin(x(2))*sin(x(4)));
B34 = 2*arm.m2*arm.s2^2*cos(2*x(4));
B35 = 0;
B36 = -8*arm.m2*arm.s2^2*cos(2*x(4));

B41 = 0;
B42 = arm.m2*arm.s2*(arm.s2*cos(x(2))*sin(x(3) - 1.57) - arm.s2*cos(x(2)) + arm.s2*cos(x(2))*cos(x(4))^2 + arm.l1*cos(x(2))*cos(x(4))*sin(x(3) - 1.57) + arm.l1*cos(x(3) - 1.57)*sin(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(4))*sin(x(2))*sin(x(4)));
B43 = 0;
B44 = 0;
B45 = 0;
B46 = 0;
 
B = [ B11 B12 B13 B14 B15 B16;
      B21 B22 B23 B24 B25 B26;
      B31 B32 B33 B34 B35 B36;
      B41 B42 B43 B44 B45 B46 ];
  
% [x_dot*x.dot ]
x_dotx_dot = [ x(5)*x(6);   % x(1)_dot*x(2)_dot
               x(5)*x(7);   % x(1)_dot*x(3)_dot
               x(5)*x(8);   % x(1)_dot*x(4)_dot
               x(6)*x(7);   % x(2)_dot*x(3)_dot
               x(6)*x(8);   % x(2)_dot*x(4)_dot
               x(7)*x(8) ]; % x(3)_dot*x(4)_dot
     
     
     
% Centrifugal
C11 = 0;
C12 = -(arm.m2*arm.s2*(arm.s2*sin(2*x(3) + 2*x(4) - 3.14) - 2*arm.s2*sin(2*x(3) - 3.14) + arm.s2*sin(2*x(3) - 2*x(4) - 3.14) + 8*arm.l1*cos(x(2))*sin(x(3) - 1.57)*sin(x(4)) + 8*arm.s2*cos(x(3) - 1.57)*sin(x(3) - 1.57)*sin(x(2)) + 8*arm.s2*cos(x(2))*cos(x(4))*sin(x(3) - 1.57)*sin(x(4)) - 8*arm.s2*cos(x(3) - 1.57)*cos(x(4))^2*sin(x(3) - 1.57)*sin(x(2))))/8;
C13 = arm.m2*arm.s2*(arm.s2*sin(x(2)) - arm.s2*cos(x(4))*sin(x(4)) - arm.s2*cos(x(4))^2*sin(x(2)) + arm.l1*cos(x(3) - 1.57)*cos(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(2))*cos(x(4))*sin(x(4)));
C14 = -arm.m2*arm.s2*sin(x(3) - 1.57)*sin(x(2))*(arm.s2 + arm.l1*cos(x(4)));

C21 = -arm.m2*arm.s2*sin(x(3) - 1.57)*(arm.l1*sin(x(2))*sin(x(4)) - arm.s2*cos(x(3) - 1.57)*cos(x(2)) + arm.s2*cos(x(4))*sin(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(2))*cos(x(4))^2);
C22 = (arm.m2*arm.s2^2*(cos(2*x(3) - 2*x(4) - 3.14) - 2*cos(2*x(3) - 3.14) + cos(2*x(3) + 2*x(4) - 3.14)))/4;
C23 = -arm.m2*arm.s2^2*cos(2*x(4));
C24 = 0;

C31 = -arm.m2*arm.s2*(arm.s2*sin(x(2)) - arm.s2*cos(x(4))^2*sin(x(2)) + arm.l1*cos(x(3) - 1.57)*cos(x(2))*sin(x(4)) + arm.s2*cos(x(3) - 1.57)*cos(x(2))*cos(x(4))*sin(x(4)));
C32 = (arm.m2*arm.s2^2*(sin(2*x(3) - 2*x(4) - 3.14) - 2*sin(2*x(3) - 3.14) + sin(2*x(3) + 2*x(4) - 3.14)))/2;
C33 = -2*arm.m2*arm.s2^2*sin(2*x(4));
C34 = 0;

C41 = -arm.m2*arm.s2*cos(x(2))*sin(x(3) - 1.57)*(arm.s2 + arm.l1*cos(x(4)));
C42 = arm.m2*arm.s2^2*(cos(2*x(3) - 2*x(4) - 3.14) - 2*cos(2*x(3) - 3.14) + cos(2*x(3) + 2*x(4) - 3.14));
C43 = 4*arm.m2*arm.s2^2*cos(2*x(4));
C44 = 0;

C = [ C11 C12 C13 C14;
      C21 C22 C23 C24;
      C31 C32 C33 C34;
      C41 C42 C43 C44 ];

  
V = B*x_dotx_dot + C*x(5:8).^2;
  
  

% Gravity  
G1 = -(981*arm.m2*(arm.s2*(sin(x(4))*(sin(x(1) + 1.57)*sin(x(3) - 1.57) - cos(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + cos(x(1) + 1.57)*cos(x(4))*sin(x(2))) + arm.l1*cos(x(1) + 1.57)*sin(x(2))))/100 - 9.81*arm.m1*arm.s1*cos(x(1) + 1.57)*sin(x(2));
G2 = -9.81*arm.m2*sin(x(1) + 1.57)*(arm.l1*cos(x(2)) + arm.s2*cos(x(2))*cos(x(4)) + arm.s2*cos(x(3) - 1.57)*sin(x(2))*sin(x(4))) - 9.81*arm.m1*arm.s1*sin(x(1) + 1.57)*cos(x(2));
G3 = 9.81*arm.m2*arm.s2*sin(x(4))*(cos(x(1) + 1.57)*cos(x(3) - 1.57) - sin(x(1) + 1.57)*cos(x(2))*sin(x(3) - 1.57));
G4 = 9.81*arm.m2*arm.s2*(cos(x(4))*(cos(x(1) + 1.57)*sin(x(3) - 1.57) + sin(x(1) + 1.57)*cos(x(3) - 1.57)*cos(x(2))) + sin(x(1) + 1.57)*sin(x(2))*sin(x(4)));


G = [ G1; G2; G3; G4 ];


Damping = arm.B*x(5:8);

f = [ x(5:8); M\(u - V - G - Damping)];





end
