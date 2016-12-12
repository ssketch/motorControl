% This function computes the MPC control output for a model of the arm
% tracking a reference state in joint, task (Cartesian), or force space. It
% employs the Multi-Parametric Toolbox MPT3. The MPT model is created by
% first linearizing the arm model (both dynamics and output) about its 
% current state estimate.
%
% ________________________________
% |                              |
% | Control                      |
% |   linearize dx/dt = f(x,u)   | 
% |   discretize dx/dt = Ax+Bu+c |              _________
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
% The function also outputs a flag. 'flag = 0' implies no problems. 'flag
% = 1' signals that the linearization failed and the function returns
% without attempting to compute the optimal control. 'flag = 2' signals
% that the computed optimal control is outside of the arm's torque limits.
% The 'params' input is optional.

function [u, flag] = control(armModel, t, ref, params)

% if necessary, set default parameter values
if nargin < 4
    params.H = armModel.Tr/armModel.Ts + 1; % MPC prediction horizon (until can next reoptimize)
    params.wP = 1e3;                        % position cost
    params.wV = 1e-1;                       % velocity cost
    params.wU = 1;                          % control cost
    params.alpha = 1e10;                    % weighting between state (pos/vel) and control costs
end

% only reoptimize if reaction time has passed
if mod(t,armModel.Tr) == 0
    u = armModel.u.val;
else
    
    % linearize arm model (and check linearization)
    [A, B, C, c, d] = linearize(armModel, ref.space);
    if ~isreal(A) || sum(sum(isnan(A))) > 0
        flag = 1;
        return
    end
    
    % discretize dynamics of arm model
    [Ad, Bd, cd] = discretize(armModel.Ts, A, B, c);
    
    % define LTI model for MPT3 package
    model = LTISystem('A',Ad,'B',Bd,'f',cd,'C',C,'g',d,'Ts',armModel.Ts);
    
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
    ctrl = MPCController(model, params.H);
    
    % simulate open-loop system to find optimal control
    u = ctrl.evaluate(armModel.x.val, 'y.reference', ref);
    
    % check that optimal control is within bounds
    if all(u >= armModel.u.min) && all(u <= armModel.u.max)
        flag = 2;
        return
    else
        flag = 0;
        armModel.u.val = u;
    end
end

end        