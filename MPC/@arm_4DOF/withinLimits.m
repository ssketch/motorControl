% This function checks that a given arm state, in joint space, does not
% violate joint limits.
function flag = withinLimits(arm, q)

lim_th1 = [arm.thLim(1,1) arm.thLim(1,2) arm.thLim(1,2) arm.thLim(1,1)];
lim_th2 = [arm.thLim(2,1) arm.thLim(2,1) arm.thLim(2,2) arm.thLim(2,2)];
if inpolygon(q(1),q(2),lim_th1,lim_th2)
    flag = 1;
else
    flag = 0;
end

end