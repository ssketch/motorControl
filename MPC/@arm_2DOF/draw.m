% This function draws the arm in its current configuration, outputting a
% frame. It assumes that the shoulder is at (0,0), the default for any
% 'arm' object.
function M = draw(arm)

% plot limbs
plot([arm.shld(1),arm.elbw(1)],[arm.shld(2),arm.elbw(2)],'b',...
     [arm.elbw(1),arm.x(1)],[arm.elbw(2),arm.x(2)],'c','LineWidth',6);
hold on

% plot joints
plot(arm.shld(1),arm.shld(2),'ko','MarkerSize',20,'MarkerFaceColor','k');
plot(arm.elbw(1),arm.elbw(2),'ko','MarkerSize',20,'MarkerFaceColor','k');
plot(arm.x(1),arm.x(2),'ko','MarkerSize',20,'MarkerFaceColor','k');

% set axes to cover workspace (depends on handedness)
if strcmp(arm.hand,'right')
    axis([-0.75*arm.l1, arm.l1+arm.l2,...
          -0.75*arm.l2+0.1, arm.l1+arm.l2+0.1]);
else
    axis([-(arm.l1+arm.l2), 0.75*arm.l1,...
          -0.75*arm.l2+0.1, arm.l1+arm.l2+0.1]);
end

xlabel('x','FontSize',24);
ylabel('y','FontSize',24);
grid on
hold off

M = getframe;

end