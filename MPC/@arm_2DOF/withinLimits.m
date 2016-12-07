% This function checks that a given arm state, in joint space, does not
% violate joint limits. If no state is specified as input, the function
% checks the current state of the 'arm' object.
function flag = withinLimits(arm, x)

% if no state specified, use current arm state
if nargin == 1
    x = arm.x.val;
end

flag = false;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% I'm just going to comment out what's not working since last update and
% writing something given the new ways things are defined in the
% constructor, which are more consistent with the controls framework.  If
% you agree with what I did, feel free to delete commented section below.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if all(q >= arm.jntLim(:,1)) && all(q <= arm.jntLim(:,2))
%     flag = true;
% end

if all( x >= arm.x.min ) && all( x <= arm.x.max )
    flag = true;
end

end