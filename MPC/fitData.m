% This function fits a line to X-Y data, outputting the slope and intercept
% of the fit.
function [slope, intercept] = fitData(X, Y)

P = polyfit(X,Y,1);
slope = P(1);
intercept = P(2);

end