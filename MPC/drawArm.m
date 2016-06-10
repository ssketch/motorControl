% This function draws arm in its current configuration, outputting a frame.
% It assumes that the shoulder is at (0,0) in Cartesian space.
function M = drawArm(th, params)

% compute forward kinematics
[~,shoulder,elbow,hand,~] = fwdKin(th, [0;0], params);

% plot limbs
plot([shoulder(1),elbow(1)],[shoulder(2),elbow(2)],'b',...
     [elbow(1),hand(1)],[elbow(2),hand(2)],'c','LineWidth',6);
hold on

% plot joints
plot(shoulder(1),shoulder(2),'ko','MarkerSize',20,'MarkerFaceColor','k');
plot(elbow(1),elbow(2),'ko','MarkerSize',20,'MarkerFaceColor','k');
plot(hand(1),hand(2),'ko','MarkerSize',20,'MarkerFaceColor','k');

% set axes to cover workspace
axis([-0.75*params.l1, params.l1+params.l2,...
      -0.75*params.l2+0.1, params.l1+params.l2+0.1]);

xlabel('x','FontSize',24);
ylabel('y','FontSize',24);
grid on
hold off

M = getframe;

end