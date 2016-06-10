% This function returns the derivative of the Jacobian for the arm in its
% current state (position & velocity).
function J_dot = jacobianDeriv(th, th_dot, params)

th1 = th(1);  th1_dot = th_dot(1);
th2 = th(2);  th2_dot = th_dot(2);
J_dot = [(-params.l1*cos(th1)*th1_dot - params.l2*cos(th1+th2)*(th1_dot+th2_dot)) -params.l2*cos(th1+th2)*(th1_dot+th2_dot);
         (-params.l1*sin(th1)*th1_dot - params.l2*sin(th1+th2)*(th1_dot+th2_dot)) -params.l2*sin(th1+th2)*(th1_dot+th2_dot)];

end