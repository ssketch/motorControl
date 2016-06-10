% This function returns the Jacobian for the arm in its current
% configuration.
function J = jacobian(th, params)

th1 = th(1);
th2 = th(2);
J = [(-params.l1*sin(th1) - params.l2*sin(th1+th2)) -params.l2*sin(th1+th2);
     (params.l1*cos(th1) + params.l2*cos(th1+th2))   params.l2*cos(th1+th2)];

end