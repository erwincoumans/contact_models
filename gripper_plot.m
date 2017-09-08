function gripper_plot(x)

% Parameters
h = 0.02;
r = 0.5;

% Disk
angles = linspace(0, 2*pi, 30);
xs = r*cos(angles);
ys = r*sin(angles);

% Plotting
lims = [-4 4 -1 5]*r;
clf
patch(lims([1 2 2 1]), [lims([3 3]) 0 0], 0.8+[0 0 0]);
h_x1 = line(x(4)+[0 0], x(6)+[0 3*r], 'Color', 'k', 'LineWidth', 2);
h_x2 = line(x(5)+[0 0], x(6)+[0 3*r], 'Color', 'k', 'LineWidth', 2);
h_ceil = patch([-2 2 2 -2]*r, x(6)+[3 3 4 4]*r, 'k');
h_disk = patch(x(1) + xs, x(2) + ys, 0.8+[0 0 0]);
h_tick = line(x(1)+[0 r*cos(x(3))], x(2)+[0 r*sin(x(3))], 'Color', 'k', 'LineStyle', '--');
axis(lims)

for k = 1:size(x,2)
    % Plotting
    h_x1.XData = x(4,k)+[0 0];
    h_x1.YData = x(6,k)+[0 3*r];
    h_x2.XData = x(5,k)+[0 0];
    h_x2.YData = x(6,k)+[0 3*r];
    h_ceil.YData = x(6,k)+[3 3 4 4]*r;
    h_disk.XData = x(1,k) + xs;
    h_disk.YData = x(2,k) + ys;
    h_tick.XData = x(1,k)+[0 r*cos(x(3,k))];
    h_tick.YData = x(2,k)+[0 r*sin(x(3,k))];
    axis(lims)
    
    pause(h)
end
