% This function returns the Jacobian for the arm in its current
% configuration.
function J = jacobian(th, params)

th1 = th(1);
th2 = th(2);
J = [(-params.L1*sind(th1) - params.L2*sind(th1+th2)) -params.L2*sind(th1+th2);
     (params.L1*cosd(th1) + params.L2*cosd(th1+th2))   params.L2*cosd(th1+th2)];

end