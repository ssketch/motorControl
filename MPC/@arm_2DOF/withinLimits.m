% This function checks that a given arm state, in joint space, does not
% violate joint limits.
function flag = withinLimits(arm, q)

flag = false;
if all(q >= arm.jntLim(:,1)) && all(q <= arm.jntLim(:,2))
    flag = true;
end

end