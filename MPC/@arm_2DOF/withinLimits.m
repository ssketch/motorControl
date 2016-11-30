% This function checks that a given arm state, in joint space, does not
% violate joint limits. If no state is specified as input, the function
% checks the current state of the 'arm' object.
function flag = withinLimits(arm, q)

% if no state specified, use current arm state
if nargin == 1
    q = arm.q;
end

flag = false;
if all(q >= arm.jntLim(:,1)) && all(q <= arm.jntLim(:,2))
    flag = true;
end

end