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
x_up = [arm.shld(1), elbw(1)];   x_lo = [elbw(1), y(1)];
y_up = [arm.shld(2), elbw(2)];   y_lo = [elbw(2), y(2)];
z_up = [arm.shld(3), elbw(3)];   z_lo = [elbw(3), y(3)];
plot3(x_up, y_up, z_up, 'b', x_lo, y_lo, z_lo, 'c', 'LineWidth',6);
hold on

% plot joints
plot3(arm.shld(1), arm.shld(2), arm.shld(3), 'ko','MarkerSize',20,'MarkerFaceColor','k');
plot3(elbw(1), elbw(2), elbw(3), 'ko','MarkerSize',20,'MarkerFaceColor','k');
plot3(y(1), y(2), y(3), 'ko','MarkerSize',20,'MarkerFaceColor','k');

% set plot parameters
axis([-2*(arm.l1 + arm.l2), 2*(arm.l1 + arm.l2), ...
      -2*(arm.l1 + arm.l2), 2*(arm.l1 + arm.l2), ...
      -0.1, 0.1]);
xlabel('x','FontSize',24);
ylabel('y','FontSize',24);
zlabel('z','FontSize',24);
view(0,90) % directly overhead since effectively 2D
grid on
hold off

M = getframe;

end
