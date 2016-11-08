% This function returns the Jacobian for the arm in a given state, taking
% handedness into account. If no state is specified as input, the function
% computes the Jacobian for the current state of the 'arm' object.
function J = jacobian(arm, q)

% if no state specified, use current state of arm
if nargin < 4
    q = arm.q;
end

% Right now I only have the parameters for a right arm
J(1,1) = arm.l4*(sin(q(4))*(cos(q(3))*sin(q(1)) - ...
    cos(q(1))*sin(q(2))*sin(q(3))) - cos(q(1))*cos(q(2))*cos(q(4))) - ...
    arm.l3*cos(q(1))*cos(q(2));
J(1,2) = sin(q(1))*(arm.l3*sin(q(2)) + arm.l4*cos(q(4))*sin(q(2)) - ...
    arm.l4*cos(q(2))*sin(q(3))*sin(q(4)));
J(1,3) = arm.l4*sin(q(4))*(cos(q(1))*sin(q(3)) - ...
    cos(q(3))*sin(q(1))*sin(q(2))); 
J(1,4) = -arm.l4*(cos(q(4))*(cos(q(1))*cos(q(3)) + ...
    sin(q(1))*sin(q(2))*sin(q(3))) - cos(q(2))*sin(q(1))*sin(q(4)));

J(2,1) = 0;
J(2,2) = arm.l4*(cos(q(2))*cos(q(4)) + sin(q(2))*sin(q(3))*sin(q(4))) + ...
    arm.l3*cos(q(2));
J(2,3) = -arm.l4*cos(q(2))*cos(q(3))*sin(q(4));
J(2,4) = -arm.l4*(sin(q(2))*sin(q(4)) + cos(q(2))*cos(q(4))*sin(q(3)));

J(3,1) = arm.l4*(sin(q(4))*(cos(q(1))*cos(q(3)) + ...
    sin(q(1))*sin(q(2))*sin(q(3))) + cos(q(2))*cos(q(4))*sin(q(1))) + ...
    arm.l3*cos(q(2))*sin(q(1));
J(3,2) = cos(q(1))*(arm.l3*sin(q(2)) + arm.l4*cos(q(4))*sin(q(2)) - ...
    arm.l4*cos(q(2))*sin(q(3))*sin(q(4)));
J(3,3) = -arm.l4*sin(q(4))*(sin(q(1))*sin(q(3)) + ...
    cos(q(1))*cos(q(3))*sin(q(2)));
J(3,4) = arm.l4*(cos(q(4))*(cos(q(3))*sin(q(1)) - ...
    cos(q(1))*sin(q(2))*sin(q(3))) + cos(q(1))*cos(q(2))*sin(q(4)));

if strcmp(arm.hand,'left')
    warning('Parameters are currently only defined for a right hand.')
end

end