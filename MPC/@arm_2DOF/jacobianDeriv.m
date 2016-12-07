% This function returns the derivative of the Jacobian for the arm in a
% given state, taking handedness into account. If no state is specified as
% input, the function computes the derivative for the current state of the
% 'arm' object.
function J_dot = jacobianDeriv(arm, x)

% if no state specified, use current arm state
if nargin == 1
    x = arm.x.val;
end

if strcmp(arm.hand,'right')
    J_dot = [(-arm.l1*cos(x(1))*x(3) - arm.l2*cos(x(1)+x(2))*(x(3)+x(4))) -arm.l2*cos(x(1)+x(2))*(x(3)+x(4));
             (-arm.l1*sin(x(1))*x(3) - arm.l2*sin(x(1)+x(2))*(x(3)+x(4))) -arm.l2*sin(x(1)+x(2))*(x(3)+x(4))];
else
    J_dot = [( arm.l1*cos(x(1))*x(3) + arm.l2*cos(x(1)+x(2))*(x(3)+x(4)))  arm.l2*cos(x(1)+x(2))*(x(3)+x(4));
             (-arm.l1*sin(x(1))*x(3) - arm.l2*sin(x(1)+x(2))*(x(3)+x(4))) -arm.l2*sin(x(1)+x(2))*(x(3)+x(4))];
end

end