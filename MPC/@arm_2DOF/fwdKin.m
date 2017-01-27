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
    warning('Posture exceeds joint limitations.')
end

% translate position
if strcmp(arm.hand,'right')
    elbw = arm.shld + [arm.l1 * cos(x(1));
                       arm.l1 * sin(x(1));
                       0];
    y(1:3,:) = elbw + [arm.l2 * cos(x(1)+x(2));
                       arm.l2 * sin(x(1)+x(2));
                       0];
else
    elbw = arm.shld + [arm.l1 * cos(pi-x(1));
                       arm.l1 * sin(pi-x(1));
                       0];
    y(1:3,:) = elbw + [arm.l2 * cos(pi-x(1)-x(2));
                       arm.l2 * sin(pi-x(1)-x(2));
                       0];
end

% translate velocity
J = jacobian(arm, x);
y(4:6,:) = J * x(3:4);

% if necessary, translate torque to force
if length(x) > 4
    
    J_red = J(1:2,1:2);
    
    % extension (-) torques
    Text = x([5,7]);
    Fext = J_red' \ (-1*Text);
    
    % flexion (+) torques
    Tflex = x([6,8]);
    Fflex = J_red' \ Tflex;
    
    % compile forces into Cartesian-space states vector
    y(7) = Fext(1);  y(8) = Fflex(1);  % y(7) + y(8) = Fx
    y(9) = Fext(2);  y(10) = Fflex(2); % y(9) + y(10) = Fy
    y(11) = 0;       y(12) = 0;        % y(11) + y(12) = Fz = 0
    
end

end