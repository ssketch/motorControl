% This function translates a given arm state from task space to joint
% space, taking arm mechanics/joint limits (i.e., can that state be
% reached?) and handedness into account. It assumes that the elbow cannot
% hyperextend (i.e., have a negative joint angle). It also outputs elbow
% position for plotting. If no state is specified as input, the function
% performs forward kinematics on the current state of the 'arm' object.
function [q, elbw, reachable] = invKin(arm, x)

% check that given state is within arm workspace, ignoring joint limits
if nargin == 2
    tooClose = abs(arm.l1-arm.l2) > norm(x(1:2));
    tooFar = abs(arm.l1+arm.l2) < norm(x(1:2));
    if tooClose || tooFar
        reachable = 0;
        q = NaN;
        elbw = NaN;
        return
    end
% if no state specified, use current arm state
else
    x = arm.x;
end

% translate position
c2 = (x(1)^2 + x(2)^2 - arm.l1^2 - arm.l2^2)/(2*arm.l1*arm.l2);
s2 = sqrt(1 - c2^2);
if strcmp(arm.hand,'right')
    q(1,:) = atan2(x(2),x(1)) - atan2(arm.l2*s2,(arm.l1+arm.l2*c2));
    q(2,:) = atan2(s2,c2);
    elbw = arm.shld + [arm.l1*cos(q(1));arm.l1*sin(q(1))];
else
    s2 = -s2;
    q(1,:) = pi - (atan2(x(2),x(1)) - atan2(arm.l2*s2,(arm.l1+arm.l2*c2)));
    q(2,:) = -atan2(s2,c2);
    elbw = arm.shld + [arm.l1*cos(pi-q(1));arm.l1*sin(pi-q(1))];
end

% translate velocity
J = jacobian(arm, q);
q(3:4,:) = J\x(3:4);

% check that resulting state is within joint limits
reachable = withinLimits(arm, q);
if ~reachable
    q = NaN;
    elbw = NaN;
    return
end

end