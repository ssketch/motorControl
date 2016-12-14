% This function returns the Jacobian for the arm in a given state, taking
% handedness into account. If no state is specified as input, the function
% computes the Jacobian for the current state of the 'arm' object.
function J = jacobian(arm, x)

% if no state specified, use current arm state
if nargin < 2
    x = arm.x.val;
end

if strcmp( arm.hand, 'right' )
    J = [(-arm.l1*sin(x(1)) - arm.l2*sin(x(1)+x(2))) -arm.l2*sin(x(1)+x(2));
         (arm.l1*cos(x(1)) + arm.l2*cos(x(1)+x(2)))   arm.l2*cos(x(1)+x(2));
         0                                            0];
else
    J = [(arm.l1*sin(x(1)) + arm.l2*sin(x(1)+x(2))) arm.l2*sin(x(1)+x(2));
         (arm.l1*cos(x(1)) + arm.l2*cos(x(1)+x(2))) arm.l2*cos(x(1)+x(2));
         0                                          0];
end

end