% This function returns the Jacobian for the arm in a given state, taking
% handedness into account. If no state is specified as input, the function
% computes the Jacobian for the current state of the 'arm' object.
function J = jacobian(arm, x)

% if no state specified, use current state of arm
if nargin < 2
    x = arm.x.val;
end

% Right now I only have the parameters for a right arm
J(1,1) = arm.l1*cos(x(1))*sin(x(2)) + arm.l2*cos(x(1))*cos(x(4))*sin(x(2)) - arm.l2*cos(x(3))*sin(x(1))*sin(x(4)) - arm.l2*cos(x(1))*cos(x(2))*sin(x(3))*sin(x(4));
J(1,2) = sin(x(1))*(arm.l1*cos(x(2)) + arm.l2*cos(x(2))*cos(x(4)) + arm.l2*sin(x(2))*sin(x(3))*sin(x(4)));
J(1,3) = -arm.l2*sin(x(4))*(cos(x(1))*sin(x(3)) + cos(x(2))*cos(x(3))*sin(x(1)));
J(1,4) = -arm.l2*(sin(x(1))*sin(x(2))*sin(x(4)) - cos(x(1))*cos(x(3))*cos(x(4)) + cos(x(2))*cos(x(4))*sin(x(1))*sin(x(3)));

J(2,1) = 0;
J(2,2) = arm.l1*sin(x(2)) + arm.l2*cos(x(4))*sin(x(2)) - arm.l2*cos(x(2))*sin(x(3))*sin(x(4));
J(2,3) = -arm.l2*cos(x(3))*sin(x(2))*sin(x(4));
J(2,4) = arm.l2*cos(x(2))*sin(x(4)) - arm.l2*cos(x(4))*sin(x(2))*sin(x(3));

J(3,1) = arm.l1*sin(x(1))*sin(x(2)) + arm.l2*cos(x(1))*cos(x(3))*sin(x(4)) + arm.l2*cos(x(4))*sin(x(1))*sin(x(2)) - arm.l2*cos(x(2))*sin(x(1))*sin(x(3))*sin(x(4));
J(3,2) = -cos(x(1))*(arm.l1*cos(x(2)) + arm.l2*cos(x(2))*cos(x(4)) + arm.l2*sin(x(2))*sin(x(3))*sin(x(4)));
J(3,3) = -arm.l2*sin(x(4))*(sin(x(1))*sin(x(3)) - cos(x(1))*cos(x(2))*cos(x(3)));
J(3,4) = arm.l2*(cos(x(3))*cos(x(4))*sin(x(1)) + cos(x(1))*sin(x(2))*sin(x(4)) + cos(x(1))*cos(x(2))*cos(x(4))*sin(x(3)));

if strcmp(arm.hand,'left')
    warning('Parameters are currently only defined for a right hand.')
end

end