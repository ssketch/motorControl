% This function translates a given arm state from joint space to cartesian
% space, taking joint limits (i.e., can that state be reached?) and
% handedness into account. It also outputs elbow position for plotting. If
% no state is specified as input, the function performs forward kinematics
% on the current state of the 'arm' object.
function [y, elbw, reachable] = fwdKin(arm, x)

% If no state is specified, use the current arm state.
if nargin == 1
    x = arm.x.val;
end

% Check that joint limits are satisfied
reachable = withinLimits(arm, x);
if ~reachable
    y = NaN;
    elbw = NaN;
    warning('Posture exceeds joint limitations.')
    return
end


%% Compute forward kinematics
% Translate joint angles into the position of the hand and the elbow.
if strcmp(arm.hand,'right')
    elbw = arm.shld + [ arm.l1 .* cos(x(1));
                        arm.l1 .* sin(x(1))];
    y(1:2,:) = elbw + [ arm.l2 .* cos(x(1)+x(2));
                        arm.l2 .* sin(x(1)+x(2))];
else
    elbw = arm.shld + [ arm.l1 .* cos(pi-x(1));
                        arm.l1 .* sin(pi-x(1))];
    y(1:2,:) = elbw + [ arm.l2 .* cos(pi-x(1)-x(2));
                        arm.l2 .* sin(pi-x(1)-x(2))];
end

% Translate joint angular velocities into the velocity of the hand and the
% elbow.
J = jacobian( arm, x );
y(3:4,:) = J * x(3:4);

end