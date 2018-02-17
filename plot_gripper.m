function plot_gripper(params, x, f, filename)

% Parameters
h = params.h;
r = params.r; % disk radius

% Disk
angles = linspace(0, 2*pi, 30);
xs = r*cos(angles);
ys = r*sin(angles);

% Plotting
lims = [-4 4 -1 5]*r;
clf
hf = gcf;
patch(lims([1 2 2 1]), [lims([3 3]) 0 0], 0.8+[0 0 0]);
h_x1 = line(x(4)+[0 0], x(6)+[0 3*r], 'Color', 'k', 'LineWidth', 2);
h_x2 = line(x(5)+[0 0], x(6)+[0 3*r], 'Color', 'k', 'LineWidth', 2);
h_ceil = patch([-2 2 2 -2]*r, x(6)+[3 3 4 4]*r, 'k');
h_disk = patch(x(1) + xs, x(2) + ys, 0.8+[0 0 0]);
h_tick = line(x(1)+[0 r*cos(x(3))], x(2)+[0 r*sin(x(3))], 'Color', 'k', 'LineStyle', '--');
h_quiv1 = [];
if (nargin >= 3)
    hold on
    h_quiv1 = quiver([0 0 x(4) x(5)], [x(6)+4*r x(6)+3*r x(2) x(2)],...
        [0 0 f(3) -f(4)], [-f(1) f(2) f(8) -f(9)], 0, 'b');
    h_quiv2 = quiver([x(1)+r x(1)-r x(1)], [x(2) x(2) 0],...
        [-f(3) f(4) -f(10)], [-f(8) f(9) f(5)], 0, 'r');
    hold off
end
axis(lims)

if (nargin < 4)
    filename = '';
end

for k = 1:size(x,2)
    % Plotting
    h_x1.XData = x(4,k)+[0 0];
    h_x1.YData = x(6,k)+[0 3*r];
    h_x2.XData = x(5,k)+[0 0];
    h_x2.YData = x(6,k)+[0 3*r];
    h_ceil.XData = 0.5*(x(4,k) + x(5,k))+[-2 2 2 -2]*r;
    h_ceil.YData = x(6,k)+[3 3 4 4]*r;
    h_disk.XData = x(1,k) + xs;
    h_disk.YData = x(2,k) + ys;
    h_tick.XData = x(1,k)+[0 r*cos(x(3,k))];
    h_tick.YData = x(2,k)+[0 r*sin(x(3,k))];
    if ~isempty(h_quiv1) && (k <= size(f,2))
        h_quiv1.XData = [0 0 x(4,k) x(5,k)];
        h_quiv1.YData = [x(6,k)+4*r x(6,k)+3*r x(2,k) x(2,k)];
        h_quiv1.UData = [0 0 f(3,k) -f(4,k)];
        h_quiv1.VData = [-f(1,k) f(2,k) f(8,k) -f(9,k)];
        h_quiv2.XData = [x(1,k)+r x(1,k)-r x(1,k)];
        h_quiv2.YData = [x(2,k) x(2,k) 0];
        h_quiv2.UData = [-f(3,k) f(4,k) -f(10,k)];
        h_quiv2.VData = [-f(8,k) f(9,k) f(5,k)];
    end
    axis(lims)
    
    if ~isempty(filename)
        drawnow
        frame = getframe(hf);
        im = frame2im(frame);
        [im_inds, color_map] = rgb2ind(im, 256);
        if (k == 1)
            imwrite(im_inds, color_map, filename, 'gif', 'Loopcount', Inf);
        else
            imwrite(im_inds, color_map, filename, 'gif', 'WriteMode', 'append');
        end
    else
        pause(h);
    end
end
end