% This function translates a given arm state from joint space to Cartesian
% space, taking joint limits (i.e., can that state be reached?) and
% handedness into account. It also outputs elbow position for plotting. If
% no state is specified as input, the function performs forward kinematics
% on the current state of the 'arm' object.
function [y, elbw, reachable] = fwdKin(arm, x)

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

% translate position
if strcmp(arm.hand,'right')
    elbw = arm.shld + [arm.l1 * cos(x(1));
                       arm.l1 * sin(x(1))];
    y(1:2,:) = elbw + [arm.l2 * cos(x(1)+x(2));
                       arm.l2 * sin(x(1)+x(2))];
else
    elbw = arm.shld + [arm.l1 * cos(pi-x(1));
                       arm.l1 * sin(pi-x(1))];
    y(1:2,:) = elbw + [arm.l2 * cos(pi-x(1)-x(2));
                       arm.l2 * sin(pi-x(1)-x(2))];
end

% translate velocity
J = jacobian(arm, x);
y(3:4,:) = J * x(3:4);

end