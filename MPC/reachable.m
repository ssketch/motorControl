% This function assumes that the shoulder is at (0,0) in Cartesian space.
function unreachable = reachable(p, params)

[th,~,~] = invKin(p,zeros(params.,[0;0],params);

% check joint limits
unreachable = 0;
if th1 < min(th1_lim)
    unreachable = 1;
elseif th1 > max(th1_lim)
    unreachable = 1;
end
if th2 < min(th2_lim)
    unreachable = 1;
elseif th2 > max(th2_lim)
    unreachable = 1;
end

end