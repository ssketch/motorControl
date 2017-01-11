function pwa = nonlin2PWA( params )

gridSize = 10;
th_dot_min = -10;
th_dot_max = 10;
A = zeros( 4, 4 );
B = [ zeros( 2, 2); eye(2)./params.tau ];

lti = [];
n = 0;

% Create a mesh of the state variables over which to linearize
progBar = waitbar(0,'What you looking at Cole?');
for i = linspace( params.th1Min, params.th1Max, gridSize )
    for j = linspace( params.th2Min, params.th2Max, gridSize )
        for k = linspace( th_dot_min, th_dot_max, gridSize )
            for l = linspace( th_dot_min, th_dot_max, gridSize )
                
                % update progress bar
                n = n+1;
                waitbar(n/gridSize^4);
    
                % Estimate nonlinear, continuous-time equations of motion
                [M,V,~] = dynamics( [i, j ], [k;l], 0, params);
                fx = @(x) x + [ x(3:4); M\(-V-(params.B*180/pi)*x(3:4)) ];
                
                % Linearize the nonlinear equations about the given current
                % state in the mesh
                A(:,1) = linearize( fx, [i;j;k;l], 1 );
                A(:,2) = linearize( fx, [i;j;k;l], 2 );
                A(:,3) = linearize( fx, [i;j;k;l], 3 );
                A(:,4) = linearize( fx, [i;j;k;l], 4 );
                
                % Create local LTI model 
                sys = LTISystem( 'A', A, 'B', B );
                
                % Define valid domain for this function
                ub = [i + (params.th1Max - params.th1Min)./(gridSize.*2),...
                      j + (params.th2Max - params.th2Min)./(gridSize.*2),...
                      k + (th_dot_max - th_dot_min)./(gridSize.*2),...
                      l + (th_dot_max - th_dot_min)./(gridSize.*2)];
                lb = [i - (params.th1Max - params.th1Min)./(gridSize.*2),...
                      j - (params.th2Max - params.th2Min)./(gridSize.*2),...
                      k - (th_dot_max - th_dot_min)./(gridSize.*2),...
                      l - (th_dot_max - th_dot_min)./(gridSize.*2)];
                R = Polyhedron( 'ub', ub, 'lb', lb );
                sys.setDomain( 'x', R );
                
                % Add local LTI to array
                lti = [ lti, sys ];
                
            end
        end
    end 
end
close(progBar)

pwa = PWASystem( lti );
