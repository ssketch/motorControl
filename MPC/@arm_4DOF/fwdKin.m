% This function translates a given arm state from joint space to task
% space, taking joint limits (i.e., can that state be reached?) and
% handedness into account. It assumes that the shoulder is at (0,0), the
% default for any 'arm' object. It also outputs elbow position for
% plotting. If no state is specified as input, the function performs
% forward kinematics on the current state of the 'arm' object.
function [x, elbow, reachable] = fwdKin(arm, x)

% if no state is specified, use current arm state
if nargin < 2
    x = arm.x.val;
end

% check that joint limits are satisfied
reachable = withinLimits(arm, x);
if ~reachable
    y = NaN;
    elbw = NaN;
    warning('Posture exceeds joint limitations.')
    return
end


% POSITION
elbow = [  -arm.l3*cos(x(2))*sin(x(1)); 
            arm.l3*sin(x(2)); 
           -arm.l3*cos(x(1))*cos(x(2))];

y(1,1) = -arm.l4*(sin(x(4))*(cos(x(1))*cos(x(3)) + sin(x(1))*sin(x(2))*sin(x(3))) + cos(x(2))*cos(x(4))*sin(x(1)));
y(2,1) = arm.l4*(cos(x(4))*sin(x(2)) - cos(x(2))*sin(x(3))*sin(x(4)));
y(3,1) = arm.l4*(sin(x(4))*(cos(x(3))*sin(x(1)) - cos(x(1))*sin(x(2))*sin(x(3))) - cos(x(1))*cos(x(2))*cos(x(4)));

y = y + elbow;

if strcmp(arm.hand,'left')
    warning('Parameters are currently only defined for a right hand.')
end
    
% VELOCITY
J = arm.jacobian(x);
x(4:6) = J*x(5:8);

end