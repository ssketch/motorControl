% This function checks that a given arm state, in joint space, does not
% violate joint limits. If no state is specified as input, the function
% checks the current state of the 'arm' object.
function flag = withinLimits(arm, x)

% if no state specified, use current arm state
if nargin < 2
    x = arm.x.val;
end

% checking state that contains joint torques
if length(x) > 4
    if (all(x >= arm.x.min) && all(x <= arm.x.max))
        flag = true;
    else
        flag = false;
    end

% checking state with only joint positions/velocities
else
   if (all(x >= arm.x.min(1:4)) && all(x <= arm.x.max(1:4)))
       flag = true;
   else
       flag = false;
   end
end

end