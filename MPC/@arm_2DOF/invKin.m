% This function translates a given arm state from Cartesian space to joint
% space, taking arm mechanics/joint limits (i.e., can that state be
% reached?) and handedness into account. It assumes that the elbow cannot
% hyperextend (i.e., have a negative joint angle). It also outputs elbow
% position for plotting. If no state is specified as input, the function
% performs inverse kinematics on the current state of the 'arm' object.
function [x, elbw, reachable] = invKin(arm, y)

% check that given state is within arm workspace, ignoring joint limits
if nargin > 1
    tooClose = abs(arm.l1-arm.l2) > norm(y(1:2));
    tooFar = abs(arm.l1+arm.l2) < norm(y(1:2));
    if tooClose || tooFar
        reachable = 0;
        x = NaN;
        elbw = NaN;
        return
    end
% if no state specified, use current arm state
else
    y = arm.y.val;
end

% translate position
c2 = (y(1)^2 + y(2)^2 - arm.l1^2 - arm.l2^2)/(2*arm.l1*arm.l2);
s2 = sqrt(1 - c2^2);
if strcmp(arm.hand,'right')
    x(1,:) = atan2(y(2),y(1)) - atan2(arm.l2*s2,(arm.l1+arm.l2*c2));
    x(2,:) = atan2(s2,c2);
    elbw = arm.shld + [arm.l1*cos(x(1));arm.l1*sin(x(1))];
else
    s2 = -s2;
    x(1,:) = pi - (atan2(y(2),y(1)) - atan2(arm.l2*s2,(arm.l1+arm.l2*c2)));
    x(2,:) = -atan2(s2,c2);
    elbw = arm.shld + [arm.l1*cos(pi-x(1));arm.l1*sin(pi-x(1))];
end

% translate velocity
J = jacobian(arm, x);
x(3:4,:) = J \ y(3:4);

% check that resulting state is within joint limits
reachable = withinLimits(arm, x);
if ~reachable
    x = NaN;
    elbw = NaN;
    return
end

end