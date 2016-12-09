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
% |        u_min <= u <= u_max   |       |          | x_sens
% |______________________________|       |          |
%                |                _______|_____     |
%                |                |           |     |
%                |________________| Estimator |_____|
%                  x_est          |___________|     
%
%
% The function outputs a flag. 'flag = 0' implies no problems and the
% computed optimal control is saved in the arm's properties. 'flag = 1'
% signals that the linearization failed and the function returns without
% attempting to compute the optimal control. 'flag = 2' signals that the
% computed optimal control is outside of the arm's torque limits; the value
% is not saved in the arm's properties.
function [u, flag] = control(armModel, x_est, ref, params)

% linearize & discretize arm model
[A, B, C, c, d] = linearize(armModel, x_est, params.space);
if ~isreal(A) || sum(sum(isnan(A))) > 0
    flag = 1;
    return
end

% define model for MPT3
model = LTISystem('A',A,'B',B,'f',c,'C',C,'g',d,'Ts',armModel.Ts);

% make model track a reference (can be time-varying)
model.y.with('reference');
model.y.reference = 'free';

% set (hard) constraints
model.x.min = armModel.x.min;
model.x.max = armModel.x.max;
model.u.min = armModel.u.min;
model.u.max = armModel.u.max;

% define cost function
nInputs = length(armModel.u.val);
nOutputs = length(armModel.y.val);
model.u.penalty = QuadFunction( params.alpha * diag(params.wU*ones(nInputs,1)) );
model.y.penalty = QuadFunction( diag(params.wP*[ones(nOutputs,1) ; ...
                                     params.wV*ones(nOutputs,1)]) );

% create MPC controller
MPC_ctrl = MPCController(model, params.H);

% simulate open-loop system to find optimal control
u = MPC_ctrl.evaluate(x, 'y.reference', ref);

% check that optimal control is within bounds
if all(u >= armModel.u.min) && all(u <= armModel.u.max)
    flag = 2;
    return
else
    flag = 0;
    armModel.u.val = u;
end

end        