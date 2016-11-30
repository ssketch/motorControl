% This function checks that a given arm state, in joint space, does not
% violate joint limits.
function flag = withinLimits(arm, q)

% lim_th1 = [arm.thLim(1,1) arm.thLim(1,2) arm.thLim(1,2) arm.thLim(1,1)];
% lim_th2 = [arm.thLim(2,1) arm.thLim(2,1) arm.thLim(2,2) arm.thLim(2,2)];
% 
% [~, on ] = inpolygon(q(1),q(2),lim_th1,lim_th2);
% if on
%     flag = 1;
% else
%     flag = 0;
% end

if nargin == 1
    q = arm.q;
end

flag = false;
if all( q >= arm.thLim(:,1)) && all( q <= arm.thLim(:,2))
        flag = true;
end


end