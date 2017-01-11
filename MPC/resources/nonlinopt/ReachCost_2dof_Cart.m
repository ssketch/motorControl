function cost = ReachCost_2dof_Cart( T, state, sim, params )
toc; tic;
cost = 0;
time = linspace( 0, sim.T, length( T(1,:)) );


[t,x] = ode45(@(t,x) dxdt(t,x,T,time,params),[0 sim.T], state.init);

cost = cost + norm( x(:,1:2) );

% for i = 1:length(t)
%     y = toCartesian( x(i,:), params);
%     
%     cost = cost + 1000* norm(( y(1:2) - state.des ) * 1 );
%     cost = cost + 1 * norm( y(3:4) );
% end

% cost = cost + 1e6 * norm( (x(1:2) - state.des(1:2)) * 10 );
% cost = cost + 1e6 * norm( x(3:4) );

% Compute controller cost
% cost = cost + 0.1 * norm( T  );

% Minimize Jerk
% cost = cost + norm( diff( y(:,3:4), 2 ));


% Enforce joint limits
if any( x(:,1) > params.th1Max ) || any( x(:,1) < params.th1Min )
    cost = cost + 1e15;
elseif any( x(:,2) > params.th2Max ) || any( x(:,2) < params.th2Min )
    cost = cost + 1e15;
end

display([ 'Cost is: ' num2str(cost)])