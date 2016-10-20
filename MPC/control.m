% This function computes the MPC control output u for an MPT model of
% the human arm tracking a reference state provided by ref and the current
% state of the system x.
function u = control(model, x, movt, ctrl)

% set constraints
if strcmp(ctrl.space,'joint')
    model.x.min = params.xMin;
    model.x.max = params.xMax;
end
model.u.min = params.uMin;
model.u.max = params.uMax;

% define cost function
model.x.penalty = QuadFunction(params.xCost);
model.u.penalty = QuadFunction(params.uCost);

% make model track a reference (can be time-varying)
model.x.with('reference');
model.x.reference = 'free';

% create MPC controller
ctrl = MPCController(model, ctrl.hrzn);

% simulate open-loop system
u = ctrl.evaluate(x, 'x.reference', movt.ref);