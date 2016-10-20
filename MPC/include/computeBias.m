% This function computes bias in joint angle, assuming a linear
% relationship between joint angle and bias.
function bias = computeBias(angle, slope, intercept)

bias = slope.*angle + intercept;

end