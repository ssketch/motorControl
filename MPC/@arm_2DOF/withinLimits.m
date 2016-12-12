% This function checks that a given arm state, in joint space, does not
% violate joint limits. If no state is specified as input, the function
% checks the current state of the 'arm' object.
function flag = withinLimits(arm, x)

% if no state specified, use current arm state
if nargin < 2
    x = arm.x.val;
end

if all(x >= arm.x.min) && all(x <= arm.x.max)
    flag = true;
else
    flag = false;
end

end