% This function checks that a given arm state, in joint space, does not
% violate joint limits.
function flag = withinLimits(arm, x)

if nargin < 2
    x = arm.x.val;
end

flag = false;
if all( x >= arm.x.min ) && all( x <= arm.x.max )
        flag = true;
end