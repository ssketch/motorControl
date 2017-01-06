% This function assumes that the shoulder is at (0,0) in Cartesian space.
function th = inv_kin(p,L1,L2)

x = p(1);
y = p(2);

c2 = (x^2 + y^2 - L1^2 - L2^2)/(2*L1*L2);
if c2 > 1
    c2 = 1; % to avoid imaginary number error below (NOTE: "unreachable" signal should be ON)
end
s2 = sqrt(1 - c2^2);

% choose between two solutions (can't have negative elbow angle)
if atan2(s2,c2) < 0
    s2 = -s2;
end
    
th1 = (atan2(y,x) - atan2(L2*s2,(L1+L2*c2)))*180/pi;
th2 = atan2(s2,c2)*180/pi;

th = [th1;th2];

end