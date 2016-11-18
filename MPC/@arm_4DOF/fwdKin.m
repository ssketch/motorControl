% This function translates a given arm state from joint space to task
% space, taking joint limits (i.e., can that state be reached?) and
% handedness into account. It assumes that the shoulder is at (0,0), the
% default for any 'arm' object. It also outputs elbow position for
% plotting. If no state is specified as input, the function performs
% forward kinematics on the current state of the 'arm' object.
function [x, elbow, reachable] = fwdKin(arm, q)

% check that given state is within joint limits
if nargin == 2
    reachable = arm.withinLimits(q);
    if ~reachable
        x = NaN;
        elbow = NaN;
        warning('Posture exceeds joint limitations.')
        return
    end
% if no state specified, use current arm state, assuming reachable
else
    q = arm.q;
    reachable = 1;
end

% POSITION
elbow = [  -arm.l3*cos(q(2))*sin(q(1)); 
            arm.l3*sin(q(2)); 
           -arm.l3*cos(q(1))*cos(q(2))];

x(1,1) = -arm.l4*(sin(q(4))*(cos(q(1))*cos(q(3)) + sin(q(1))*sin(q(2))*sin(q(3))) + cos(q(2))*cos(q(4))*sin(q(1)));
x(2,1) = arm.l4*(cos(q(4))*sin(q(2)) - cos(q(2))*sin(q(3))*sin(q(4)));
x(3,1) = arm.l4*(sin(q(4))*(cos(q(3))*sin(q(1)) - cos(q(1))*sin(q(2))*sin(q(3))) - cos(q(1))*cos(q(2))*cos(q(4)));

x = x + elbow;

if strcmp(arm.hand,'left')
    warning('Parameters are currently only defined for a right hand.')
end
    
% VELOCITY
J = arm.jacobian(q);
x(4:6) = J*q(5:8);

end