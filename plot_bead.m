function plot_bead(params, x, f, filename)

% Parameters
h = params.h;
r = params.r; % disk radius
w = params.w; % slot half-width

% Disk
angles = linspace(0, 2*pi, 30);
xs = r*cos(angles);
ys = r*sin(angles);

% Plotting
lims = [-1 1 -1 1]*6*w;
clf
hf = gcf;
patch(2*lims([1 2 2 1]), [2*lims([3 3]) -w -w], 0.8+[0 0 0]);
patch(2*lims([1 2 2 1]), [ w  w 2*lims([4 4])], 0.8+[0 0 0]);

h_disk = patch(x(1) + xs, x(2) + ys, 0.8+[0 0 0]);
h_line = line([x(1) lims(2)], [x(2) 0], 'Color', 'k', 'Linewidth', 2);

h_quiv1 = [];
if (nargin >= 3)
    hold on
    f([1,3],:) = -f([1,3],:);
    h_quiv1 = quiver(x(1)+[0;0], [w;-w], [0;0], [0;0], 0, 'b');
    h_quiv2 = quiver(x(1)+[0;0], x(2)+[r;-r], [0;0], [0;0], 0, 'r');
    hold off
end
axis(lims)

if (nargin < 4)
    filename = '';
end

for k = 1:size(x,2)
    % Plotting
    h_disk.XData = x(1,k) + xs;
    h_disk.YData = x(2,k) + ys;
    h_line.XData(1) = x(1,k);
    h_line.YData(1) = x(2,k);
    if ~isempty(h_quiv1) && (k > 1)
        h_quiv1.XData = x(1,k)+[0;0];
        h_quiv1.UData = -f(3:4,k);
        h_quiv1.VData = -f(1:2,k);
        h_quiv2.XData = x(1,k)+[0;0];
        h_quiv2.YData = x(2,k)+[r;-r];
        h_quiv2.UData = f(3:4,k);
        h_quiv2.VData = f(1:2,k);
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