% This function displays the results of simulation (i.e., the output from
% 'simulate.m'). It creates a movie of the arm movement, plots position/
% velocity in Cartesian space, and plots joint torques/hand forces over
% time. All figures are saved in EPS format.
function plotResults(results, movemnt, params)

% extract results from simulation
th = results.xAct(1:2,:);
th_dot = results.xAct(3:4,:);
th_est = results.xEst(1:2,:);
th_dot_est = results.xEst(3:4,:);
T = results.T;
tsim = results.t;

% create & save movie of arm movement
figure()
for i = 1:5:length(th)
    M(i) = drawArm(th(:,i), params);
end
save('./results/reachingMovie','M');

% downsample data for plotting
p_a = zeros(2,length(th));
v_a = zeros(2,length(th));
p_e = zeros(2,length(th));
v_e = zeros(2,length(th));
for i = 1:length(th)
    [p_a(:,i),~,~,~,v_a(:,i)] = fwdKin(th(:,i),th_dot(:,i),params);
    [p_e(:,i),~,~,~,v_e(:,i)] = fwdKin(th_est(:,i),th_dot_est(:,i),params);
end
f_down = 5;
t = downsample(tsim',f_down)';
p_a = downsample(p_a',f_down)';
v_a = downsample(v_a',f_down)';
p_e = downsample(p_e',f_down)';
v_e = downsample(v_e',f_down)';
T = downsample(T',f_down)';

toDeg = 180/pi;
th_a = downsample(th',f_down)'*toDeg;
th_dot_a = downsample(th_dot',f_down)'*toDeg;
th_e = downsample(th_est',f_down)'*toDeg;
th_dot_e = downsample(th_dot_est',f_down)'*toDeg;

% plot position and velocity
figure()
plot(p_a(1,:),p_a(2,:),'b',p_e(1,:),p_e(2,:),'c--','LineWidth',1.5);
hold on
plot(params.p_i(1),params.p_i(2),'gs','MarkerSize',20,'MarkerFaceColor','g');
if (movemnt == 0)
    plot(params.p_f(1),params.p_f(2),'rx','MarkerSize',20,'LineWidth',4);
elseif (movemnt == 1)
    plot(params.p_f(1),params.p_f(2),'rx','MarkerSize',20,'LineWidth',4);
elseif (movemnt == 2)
    plot(params.p_atTurn(1),params.p_atTurn(2),'bo','MarkerSize',20,'MarkerFaceColor','b');
elseif (movemnt == 3)
    plot(params.x_ell,params.y_ell,'r:','MarkerSize',20,'LineWidth',2);
end
axis([-0.75*params.l1, params.l1+params.l2,...
      -0.75*params.l2+0.1, params.l1+params.l2+0.1]);
axis equal
grid on
title('Position','FontSize',22);
xlabel('x','FontSize',20);
ylabel('y','FontSize',20);
legend('actual','estimated','Location','NorthEast');
export_fig './results/posTraj' -eps
hold off

figure()
quiver(p_a(1,:),p_a(2,:),v_a(1,:),v_a(2,:),'b','LineWidth',1);
hold on
quiver(p_e(1,:),p_e(2,:),v_e(1,:),v_e(2,:),'c','LineWidth',1);
plot(params.p_i(1),params.p_i(2),'gs','MarkerSize',20,'MarkerFaceColor','g');
if (movemnt == 0)
    plot(params.p_f(1),params.p_f(2),'rx','MarkerSize',20,'LineWidth',4);
elseif (movemnt == 1)
    plot(params.p_f(1),params.p_f(2),'rx','MarkerSize',20,'LineWidth',4);
elseif (movemnt == 2)
    plot(params.p_atTurn(1),params.p_atTurn(2),'bo','MarkerSize',20,'MarkerFaceColor','b');
elseif (movemnt == 3)
    plot(params.x_ell,params.y_ell,'r:','MarkerSize',20,'LineWidth',2);
end
axis([-0.75*params.l1, params.l1+params.l2,...
      -0.75*params.l2+0.1, params.l1+params.l2+0.1]);
axis equal
grid on
title('Velocity','FontSize',22);
xlabel('x','FontSize',20);
ylabel('y','FontSize',20);
legend('actual','estimated','Location','NorthEast');
export_fig './results/velTraj' -eps
hold off

figure()
subplot(2,1,1)
plot(t,th_a,t,th_e,'LineWidth',3);
grid on
ylabel('Position [deg]','FontSize',14);
legend('act, shoulder','act, elbow',...
    'est, shoulder','est, elbow','Location','NorthEast');
subplot(2,1,2)
plot(t,th_dot_a,t,th_dot_e,'LineWidth',3);
grid on
xlabel('t [sec]','FontSize',20);
ylabel('Velocity [^d^e^g/_s_e_c]','FontSize',12);
legend('act, shoulder','act, elbow',...
    'est, shoulder','est, elbow','Location','NorthEast');
export_fig './results/timePlots' -eps

% plot joint torques
figure()
plot(t,T(1,:),'r',t,T(2,:),'b','LineWidth',3);
grid on
title('Joint Torques','FontSize',22);
xlabel('t [sec]','FontSize',20);
ylabel('T [Nm]','FontSize',20);
legend('shoulder','elbow','Location','Best')
export_fig './results/commTraj' -eps

end