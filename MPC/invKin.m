% If the desired point is in the workspace, this function translates
% Cartesian position, velocity, and acceleration of the arm to joint-space
% coordinates, accounting for joint limits. It assumes that the shoulder is
% at (0,0) in Cartesian space, and that the elbow cannot hyperextend (i.e.,
% have a negative joint angle).
function [inWS, th, th_dot, th_Dot] = invKin(p, v, a, params)

x = p(1);
y = p(2);

c2 = (x^2 + y^2 - params.l1^2 - params.l2^2)/(2*params.l1*params.l2);
if c2 > 1
    c2 = 1; % to avoid imaginary number error below
end
s2 = sqrt(1 - c2^2);

% choose between two solutions (can't have negative elbow angle)
if atan2(s2,c2) < 0
    s2 = -s2;
end
    
th1 = atan2(y,x) - atan2(params.l2*s2,(params.l1+params.l2*c2));
th2 = atan2(s2,c2);

% check joint limits
inWS = 1;
if th1 < min(params.th_lim(1,:))
    th1 = min(params.th_lim(1,:));
elseif th1 > max(params.th_lim(1,:))
    th1 = max(params.th_lim(1,:));
end
if th2 < min(params.th_lim(2,:))
    th2 = min(params.th_lim(2,:));
elseif th2 > max(params.th_lim(2,:))
    th2 = max(params.th_lim(2,:));
end

if inWS 
    % JOINT ANGLES
    th = [th1;th2];
    
    % JOINT VELOCITIES
    J = jacobian(th, params);
    th_dot = J\v;
    
    % JOINT ACCELERATIONS
    J_dot = jacobianDeriv(th, th_dot, params);
    th_Dot = J\(a - J_dot*th_dot);
else
    th = NaN;
    th_dot = NaN;
    th_Dot = NaN;
end

end