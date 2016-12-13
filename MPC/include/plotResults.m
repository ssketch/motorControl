% This function displays the results of simulation. It creates a movie of
% the arm movement, plots position/velocity in joint/Cartesian space, and
% plots joint torques over time. All figures are saved in EPS format.
% NOTE: Currently, this only supports 2-DOF arm planar movements.
function plotResults(arm, data)

% define parameters
nJoints = length(arm.q.val);
nStates = length(arm.x.val);
nOutputs = length(arm.y.val);
n = length(data.t);
toDeg = 180/pi;

% create & save movie of arm movement
figure()
for i = 1:n
    M(i) = draw(arm, data.x.act(:,i));
end
save('./results/reachingMovie','M');

% downsample data for plotting
f_down = 2;
t = downsample(data.t',f_down)';
u = downsample(data.u',f_down)';
q = downsample(data.q.act',f_down)'*toDeg;
x = downsample(data.x.act',f_down)'*toDeg;
qdot = x(nJoints+1:nStates,:);
y = downsample(data.y.act',f_down)';
px = y(1,:);
py = y(2,:);
vx = y(nOutputs/2+1,:);
vy = y(nOutputs/2+2,:);
q_est = downsample(data.q.est',f_down)'*toDeg;
x_est = downsample(data.x.est',f_down)'*toDeg;
qdot_est = x_est(nJoints+1:nStates,:);
y_est = downsample(data.y.est',f_down)';
px_est = y_est(1,:);
py_est = y_est(2,:);
vx_est = y_est(nOutputs/2+1,:);
vy_est = y_est(nOutputs/2+2,:);

% plot position and velocity over time
figure()
subplot(2,1,1)
plot(t,q,t,q_est,'LineWidth',3);
grid on
ylabel('Position [deg]','FontSize',14);
legend('act, shoulder','act, elbow',...
    'est, shoulder','est, elbow','Location','NorthEast');
subplot(2,1,2)
plot(t,qdot,t,qdot_est,'LineWidth',3);
grid on
xlabel('t [sec]','FontSize',20);
ylabel('Velocity [^d^e^g/_s_e_c]','FontSize',12);
legend('act, shoulder','act, elbow',...
    'est, shoulder','est, elbow','Location','NorthEast');
export_fig './results/timePlots' -eps

% plot position and velocity in space
figure()
plot(px,py,'b',px_est,py_est,'c--','LineWidth',1.5);
axis equal
grid on
title('Position','FontSize',22);
xlabel('x','FontSize',20);
ylabel('y','FontSize',20);
legend('actual','estimated','Location','NorthEast');
export_fig './results/posTraj' -eps
hold off

figure()
quiver(px(1,:),py(2,:),vx(1,:),vy(2,:),'b','LineWidth',1);
hold on
quiver(px_est(1,:),py_est(2,:),vx_est(1,:),vy_est(2,:),'c','LineWidth',1);
axis equal
grid on
title('Velocity','FontSize',22);
xlabel('x','FontSize',20);
ylabel('y','FontSize',20);
legend('actual','estimated','Location','NorthEast');
export_fig './results/velTraj' -eps
hold off

% plot joint torques over time
figure()
plot(t,u(1,:),'r',t,u(2,:),'b','LineWidth',3);
grid on
title('Joint Torques','FontSize',22);
xlabel('t [sec]','FontSize',20);
ylabel('T [Nm]','FontSize',20);
legend('shoulder','elbow','Location','Best')
export_fig './results/commTraj' -eps

end