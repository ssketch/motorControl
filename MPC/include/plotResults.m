% This function displays the results of simulation. It creates a movie of
% the arm movement, plots position/velocity in joint/Cartesian space, and
% plots joint torques over time. The 'planar' parameter views 3D
% Cartesian plots in the top-down 2D plane. All figures are saved in EPS
% format.
function plotResults(arm, data, planar)

clc

% define parameters
toRad = pi/180;
n = length(data.t);
nJoints = length(arm.q.val);
nStatesTsk = length(arm.y.val);

% create & save movie of arm movement
figure()
for i = 1:n
    M(i) = draw(arm, data.xAct(:,i)*toRad);
end
save('./results/reachingMovie','M');

% downsample data for plotting
fields = fieldnames(data);
f_down = 2;
for i = 1:length(fields)
    data.(fields{i}) = downsample(data.(fields{i})',f_down)';
end

% parse out variables to plot
data.qdotAct = data.xAct(nJoints+1:end,:);
data.qdotEst = data.xEst(nJoints+1:end,:);
data.pAct = data.yAct(1:nStatesTsk/2,:);
data.pEst = data.yEst(1:nStatesTsk/2,:);
data.vAct = data.yAct(nStatesTsk/2+1:end,:);
data.vEst = data.yEst(nStatesTsk/2+1:end,:);

% plot position and velocity over time
figure()
subplot(2,1,1)
plot(data.t,data.qAct,data.t,data.qEst,'LineWidth',3); 
grid on
ylabel('Position [deg]','FontSize',14);
actNames = cell(nJoints,1); actNames(:) = {', act'};
estNames = cell(nJoints,1); estNames(:) = {', est'};
legendNames = strcat([arm.DOFs;arm.DOFs], [actNames;estNames]);
legend(legendNames,'Location','NorthEast');
subplot(2,1,2)
plot(data.t,data.qdotAct,data.t,data.qdotEst,'LineWidth',3);
grid on
xlabel('t [sec]','FontSize',16);
ylabel('Velocity [^d^e^g/_s_e_c]','FontSize',12);
export_fig './results/timePlots' -eps

% plot position and velocity in space
figure()
plot3(data.pAct(1,:),data.pAct(2,:),data.pAct(3,:),'b','LineWidth',1.5);
hold on
plot3(data.pEst(1,:),data.pEst(2,:),data.pEst(3,:),'c--','LineWidth',1.5);
axis equal
grid on
if planar
    view(0,90)
end
title('Position','FontSize',22);
xlabel('x','FontSize',20);
ylabel('y','FontSize',20);
zlabel('z','FontSize',20);
legend('actual','estimated','Location','NorthEast');
export_fig './results/posTraj' -eps
hold off

figure()
quiver3(data.pAct(1,:),data.pAct(2,:),data.pAct(3,:),...
    data.vAct(1,:),data.vAct(2,:),data.vAct(3,:),'b','LineWidth',1);
hold on
quiver3(data.pEst(1,:),data.pEst(2,:),data.pEst(3,:),...
    data.vEst(1,:),data.vEst(2,:),data.vEst(3,:),'c--','LineWidth',1);
axis equal
grid on
if planar
    view(0,90)
end
title('Velocity','FontSize',22);
xlabel('x','FontSize',20);
ylabel('y','FontSize',20);
zlabel('z','FontSize',20);
legend('actual','estimated','Location','NorthEast');
export_fig './results/velTraj' -eps
hold off

% plot joint torques over time
figure()
plot(data.t,data.u,'LineWidth',3);
grid on
title('Joint Torques','FontSize',22);
xlabel('t [sec]','FontSize',20);
ylabel('T [Nm]','FontSize',20);
uLegend = {};
for i = 1:nJoints
    currJnt = strcat(repmat(arm.DOFs(i),2,1),{' -';' +'});
    uLegend = [uLegend ; currJnt];
end
legend(uLegend,'Location','NorthEast');
export_fig './results/commTraj' -eps

end