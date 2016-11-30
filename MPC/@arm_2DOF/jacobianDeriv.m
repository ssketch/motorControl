% This function returns the derivative of the Jacobian for the arm in a
% given state, taking handedness into account. If no state is specified as
% input, the function computes the derivative for the current state of the
% 'arm' object.
function J_dot = jacobianDeriv(arm, q)

% if no state specified, use current arm state
if nargin == 1
    q = arm.q;
end

if strcmp(arm.hand,'right')
    J_dot = [(-arm.l1*cos(q(1))*q(3) - arm.l2*cos(q(1)+q(2))*(q(3)+q(4))) -arm.l2*cos(q(1)+q(2))*(q(3)+q(4));
             (-arm.l1*sin(q(1))*q(3) - arm.l2*sin(q(1)+q(2))*(q(3)+q(4))) -arm.l2*sin(q(1)+q(2))*(q(3)+q(4))];
else
    J_dot = [( arm.l1*cos(q(1))*q(3) + arm.l2*cos(q(1)+q(2))*(q(3)+q(4)))  arm.l2*cos(q(1)+q(2))*(q(3)+q(4));
             (-arm.l1*sin(q(1))*q(3) - arm.l2*sin(q(1)+q(2))*(q(3)+q(4))) -arm.l2*sin(q(1)+q(2))*(q(3)+q(4))];
end

end