% This function computes bias and noise in joint angle, assuming a linear
% relationship between joint angle and bias.
function bias = computeError(arm)

bias = slope.*angle + intercept;

end