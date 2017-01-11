function [slope,intercept] = fit_data(X,Y)

P = polyfit(X,Y,1);
slope = P(1);
intercept = P(2);

end