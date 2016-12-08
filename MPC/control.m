% This function computes the MPC control output u for a model of the arm
% tracking a reference state ref in either joint or task (Cartesian) space.
% It employs the Multi-Parametric Toolbox MPT3. The MPT model is created by
% first linearizing the arm model (both dynamics and output) about the 
% current state estimate.
%
% ________________________________
% |                              |
% | Control                      |
% |   linearize dx/dt = f(x,u)   |              _________
% |   linearize y = g(x)         |       u*     |       |   x
% |                              |______________| Plant |____
% |   min  y'Qy + u'Ru           |       |      |_______|  
% |   s.t. dx/dt = Ax + Bu + f   |       |          |
% |        y = Cx + g            |       |          |  
% |        x_min <= x <= x_max   |       |          |
% |        u_min <= u <= u_max   |       |          |
% |______________________________|       |          | y
%                |                _______|_____     |
%                |                |           |     |
%                |________________| Estimator |_____|
%                  x_est          |___________|     

function [u, flag] = control(arm, x, u, space, ref)

% linearize & discretize arm model
[A, B, C, c, d] = linearize(arm, x, u, space);

% define model for MPT3
model = LTISystem('A',A,'B',B,'f',c,'C',C,'g',d,'Ts',arm.Ts);

% make model track a reference (can be time-varying)
model.y.with('reference');
model.y.reference = 'free';

% set (hard) constraints
model.x.min = arm.x.min;
model.x.max = arm.x.max;
model.u.min = arm.u.min;
model.u.max = arm.u.max;

% define cost function
model.y.penalty = QuadFunction( diag(1e3*[ones(length(arm.q.min),1); ...
            1e-1*ones(length(arm.q.min),1)]));
model.u.penalty = QuadFunction( diag(ones(length(u),1)));

% create MPC controller
MPC_ctrl = MPCController(model, ctrl.H);

% simulate open-loop system
u = MPC_ctrl.evaluate(x, 'y.reference', ref);

% check that optimal control is within bounds
if (Tcurr(1) < params.torq_lim(1,1) || Tcurr(1) > params.torq_lim(1,2)) ...
        || (Tcurr(2) < params.torq_lim(2,1) || Tcurr(2) > params.torq_lim(2,2))
    flag = 1;
    return
end

end        