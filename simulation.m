% Parameters
h = 0.02;
r = 0.5;

% Disk
angles = linspace(0, 2*pi, 30);
xs = r*cos(angles);
ys = r*sin(angles);

% Initial pose
x = [0, r, 0, 2*r, -2*r, 0, zeros(1,6)]';

% Plotting
lims = [-4 4 -1 5]*r;
ax = axes(); hold on;
patch(lims([1 2 2 1]), [lims([3 3]) 0 0], 0.8+[0 0 0]);
h_x1 = line(x(4)+[0 0], x(6)+[0 3*r], 'Color', 'k', 'LineWidth', 2);
h_x2 = line(x(5)+[0 0], x(6)+[0 3*r], 'Color', 'k', 'LineWidth', 2);
h_ceil = patch([-2 2 2 -2]*r, x(6)+[3 3 4 4]*r, 'k');
h_disk = patch(x(1) + xs, x(2) + ys, 0.8+[0 0 0]);
h_tick = line(x(1)+[0 r*cos(x(3))], x(2)+[0 r*sin(x(3))], 'Color', 'k', 'LineStyle', '--');
axis(lims)

% Fixed control input
u = [-4 4 2]';

for k = 1:100
    % Dynamics
    x = gripper_sim(x, u);
    
    % Plotting
    h_x1.XData = x(4)+[0 0];
    h_x1.YData = x(6)+[0 3*r];
    h_x2.XData = x(5)+[0 0];
    h_x2.YData = x(6)+[0 3*r];
    h_ceil.YData = x(6)+[3 3 4 4]*r;
    h_disk.XData = x(1) + xs;
    h_disk.YData = x(2) + ys;
    h_tick.XData = x(1)+[0 r*cos(x(3))];
    h_tick.YData = x(2)+[0 r*sin(x(3))];
    axis(lims)
    
    pause(h)
end
