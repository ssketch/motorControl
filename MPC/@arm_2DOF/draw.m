% This function draws the arm in state x, outputting a frame. If no state
% is specified as input, the function draws the current state of the 'arm'
% object.
function M = draw(arm, x)

% if no state is specified, use current arm state
if nargin < 2
    x = arm.x.val;
end

% run forward kinematics to guarantee correspondence between spaces
[y, elbw, ~] = fwdKin(arm, x);

% plot limbs
plot([arm.shld(1), elbw(1)], [arm.shld(2), elbw(2)], 'b',...
     [arm.elbw(1), y(1)], [arm.elbw(2), y(2)], 'c',...
     'LineWidth',6 );
hold on

% plot joints
plot(arm.shld(1), arm.shld(2), 'ko','MarkerSize',20,'MarkerFaceColor','k');
plot(elbw(1), elbw(2), 'ko','MarkerSize',20,'MarkerFaceColor','k');
plot(y(1), y(2), 'ko','MarkerSize',20,'MarkerFaceColor','k');

% set plot parameters
axis([-2*(arm.l1 + arm.l2), 2*(arm.l1 + arm.l2), ...
      -2*(arm.l1 + arm.l2), 2*(arm.l1 + arm.l2)]);
xlabel('x','FontSize',24);
ylabel('y','FontSize',24);
grid on
hold off

M = getframe;

end
