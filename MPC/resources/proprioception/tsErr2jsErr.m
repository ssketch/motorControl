% This function translates a task-space error data into joint space values
% using RR inverse kinematics.
function [Th_targ,dTh_bias,dTh_nois] = tsErr2jsErr(X_targ,X_cent,X_maj,L1,L2)

% convert task-space error data into joint space
Th_targ = inv_kin(X_targ,L1,L2);
Th_cent = inv_kin(X_cent,L1,L2);
Th_maj = inv_kin(X_maj,L1,L2);

% compute joint-space error data
dTh_bias = Th_targ - Th_cent; % bias = perceived - actual
dTh_nois = Th_maj - Th_cent;

end