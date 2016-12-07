% This function checks that a given arm state, in joint space, does not
% violate joint limits. If no state is specified as input, the function
% checks the current state of the 'arm' object.
function flag = withinLimits(arm, x)

% if no state specified, use current arm state
if nargin == 1
    x = arm.x;
end

flag = false;
if all(x >= arm.xLim(:,1)) && all(x <= arm.xLim(:,2))
    flag = true;
end

end