% This function draws the arm in its current configuration, outputting a
% frame. It assumes that the shoulder is at (0,0), the default for any
% 'arm' object.
function M = draw(arm)
% Run forwad kinematics just to be sure
[ arm.x, arm.elbow ] = fwdKin( arm, arm.q );

% plot limbs
plot3([arm.shld(1), arm.elbow(1)], [arm.shld(2), arm.elbow(2)], ...
      [arm.shld(3), arm.elbow(3)], 'b', 'LineWidth', 6 )
hold on
plot3([arm.elbow(1), arm.x(1)], [arm.elbow(2), arm.x(2)], ...
      [arm.elbow(3), arm.x(3)], 'c', 'LineWidth', 6 )

% plot joints
plot3( arm.shld(1), arm.shld(2), arm.shld(3), ...
    'ko', 'MarkerSize', 20, 'MarkerFaceColor', 'k');
plot3( arm.elbow(1), arm.elbow(2), arm.elbow(3), ...
    'ko', 'MarkerSize', 20, 'MarkerFaceColor', 'k');
plot3( arm.x(1), arm.x(2), arm.x(3), ...
    'ko','MarkerSize',20,'MarkerFaceColor','k');

% set axes to cover workspace (depends on handedness)
% if strcmp(arm.hand,'right')
%     axis([-0.75*arm.l1, arm.l1+arm.l2,...
%           -0.75*arm.l2+0.1, arm.l1+arm.l2+0.1]);
% else
%     axis([-(arm.l1+arm.l2), 0.75*arm.l1,...
%           -0.75*arm.l2+0.1, arm.l1+arm.l2+0.1]);
% end

xlabel('x','FontSize',24);
ylabel('y','FontSize',24);
zlabel('z', 'FontSize', 24);
grid on
hold off

M = getframe;

end