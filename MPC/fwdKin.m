% This function translates joint-space position and velocity of the arm to
% Cartesian coordinates. It assumes that the shoulder is at (0,0) in
% Cartesian space and also outputs coordinates for the shoulder, elbow, and
% hand.
function [p, shoulder, elbow, hand, v] = fwdKin(th, th_dot, params)

th1 = th(1);
th2 = th(2);

% POSITION
shoulder = [0;0];
elbow = shoulder + [params.l1*cos(th1);params.l1*sin(th1)];
hand = elbow + [params.l2*cos(th1+th2);params.l2*sin(th1+th2)];
p = hand;

% VELOCITY
J = jacobian(th, params);
v = J*th_dot;

end