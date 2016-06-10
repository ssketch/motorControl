% This function computes the MPC control output T for a nonlinear model of
% the human arm tracking a reference state provided by ref and the current
% state of the system x.
function T = control(model, x, ref, params)

% set constraints
if params.constrainX
    model.x.min = params.xMin;
    model.x.max = params.xMax;
end
if params.constrainU
    model.u.min = params.uMin;
    model.u.max = params.uMax;
end

% define cost function
model.x.penalty = QuadFunction(params.xCost);
model.u.penalty = QuadFunction(params.uCost);

% make model track a reference (can be time-varying)
model.x.with('reference');
model.x.reference = 'free';

% create MPC controller
ctrl = MPCController(model, params.Nhrzn);

% simulate open-loop system
T = ctrl.evaluate(x, 'x.reference', ref);