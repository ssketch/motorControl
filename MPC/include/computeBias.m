% This function computes bias in joint angle for the current arm state,
% assuming that bias varies linearly with joint angle.
function bias = computeBias(arm)

m = arm.sensBias.slope;
b = arm.sensBias.inter;

bias = m .* arm.q + b;

end