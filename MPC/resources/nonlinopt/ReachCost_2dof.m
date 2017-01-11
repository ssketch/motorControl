function cost = ReachCost_2dof( T, state, sim, params )
toc; tic;
cost = 0;
time = linspace( 0, sim.T, length( T(1,:)) );
% time = time ./ sim.Ts;

% Iterate the system forward in time
% state.current = state.init;
% state.history = state.current;
% state.control = T;
% for i = 1:ceil(sim.T/sim.Ts)
%     
%     [~,y] = ode45(@(t,x) dxdt(t,x,T,time,params),[0 sim.Ts], state.current);
%     
%     state.current = y(end,:)';
%     state.history = [state.history, state.current ];
%     
%     % Compute stage cost
%     cost = cost + 500 * norm( state.current - state.des );
%     
% end


[t,y] = ode45(@(t,x) dxdt(t,x,T,time,params),[0 sim.T], state.init);

for i = 1:length(t)
    cost = cost + 1 * norm( y(i,1:2) - state.des(1:2)' );
    cost = cost + 1e-6 * norm( y(i,3:4) - state.des(3:4)' );
end

% cost = cost + norm( y(end,:) - state.des' );

% Compute controller cost
cost = cost + 1e-3 * norm( T );

% Compute stage cost
% cost = cost + 500 * norm( state.current - state.des );

if any( y(:,1) > params.th1Max ) || any( y(:,1) < params.th1Min )
    cost = cost + 1e15;
elseif any( y(:,2) > params.th2Max ) || any( y(:,2) < params.th2Min )
    cost = cost + 1e15;
end

display([ 'Cost is: ' num2str(cost)])