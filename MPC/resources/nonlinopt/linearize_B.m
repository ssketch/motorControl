function [ slope ] = linearize_B( fx, x, Ts, params )

% if isa( fx, 'function_handle' )

% for i = 1:length( x) 
%     
%     eps = 1e-6;
%     x_eps = x;
%     x_eps(i) = x(i) + eps;
% 
%     slope(:,i) = ( fx(x_eps) - fx(x)) ./ eps;
% 
% end
% 
% 
% const = fx(x) - slope * x * Ts;


% Compute mass matrix
a1 = params.I1 + params.I2 + params.m2*params.L1^2;
a2 = params.m2 * params.L1 * params.Lc2;
a3 = params.I2;

M11 = a1 + 2*a2*cos(x(2));
M12 = a3 + a2*cos(x(2));
M21 = M12;
M22 = a3;
M = [M11 M12;
     M21 M22];
 
slope = [zeros(2,2); inv(M)];