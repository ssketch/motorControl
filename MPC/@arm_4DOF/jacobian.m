% This function returns the Jacobian for the arm in a given state, taking
% handedness into account. If no state is specified as input, the function
% computes the Jacobian for the current state of the 'arm' object.
function J = jacobian(arm, q)

% if no state specified, use current state of arm
if nargin < 2
    q = arm.q;
end

if strcmp(arm.hand,'right')
    J = [(-arm.l1*sin(q(1)) - arm.l2*sin(q(1)+q(2))) -arm.l2*sin(q(1)+q(2));
         (arm.l1*cos(q(1)) + arm.l2*cos(q(1)+q(2)))   arm.l2*cos(q(1)+q(2))];
else
    J = [(arm.l1*sin(q(1)) + arm.l2*sin(q(1)+q(2))) arm.l2*sin(q(1)+q(2));
         (arm.l1*cos(q(1)) + arm.l2*cos(q(1)+q(2))) arm.l2*cos(q(1)+q(2))];
end

end