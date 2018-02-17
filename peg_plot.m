function peg_plot(params, x, f, filename)

% Parameters
h = params.h;
r = params.r; % peg radius
l = params.l; % peg half-length
w = params.w; % slot half-width

xs = [-1  1  1 -1]*l;
ys = [ 1  1 -1 -1]*r;

% Plotting
lims = [-1 1 -1 1]*6*w;
clf
hf = gcf;
patch(2*lims([1 2 2 1]), [2*lims([3 3]) -w -w], 0.8+[0 0 0]);
patch(2*lims([1 2 2 1]), [ w  w 2*lims([4 4])], 0.8+[0 0 0]);

h_disk = patch(x(1) + xs*cos(x(3)) - ys*sin(x(3)), ...
               x(2) + xs*sin(x(3)) + ys*cos(x(3)), ...
               0.8+[0 0 0]);
h_quiv1 = [];
if (nargin >= 3)
    hold on
    f([1:2,5:6],:) = -f([1:2,5:6],:);
    h_quiv1 = quiver(h_disk.XData, [1 1 -1 -1]'*w, [0 0 0 0]', [0 0 0 0]', 0, 'b');
    h_quiv2 = quiver(h_disk.XData, h_disk.YData, [0 0 0 0]', [0 0 0 0]', 0, 'r');
    hold off
end
axis(lims)

if (nargin < 4)
    filename = '';
end

for k = 1:size(x,2)
    % Plotting
    h_disk.XData = x(1,k) + xs*cos(x(3,k)) - ys*sin(x(3,k));
    h_disk.YData = x(2,k) + xs*sin(x(3,k)) + ys*cos(x(3,k));
    if ~isempty(h_quiv1) && (k > 1)
        h_quiv1.XData = h_disk.XData;
        h_quiv1.UData = -f(5:8,k);
        h_quiv1.VData = -f(1:4,k);
        h_quiv2.XData = h_disk.XData;
        h_quiv2.YData = h_disk.YData;
        h_quiv2.UData = f(5:8,k);
        h_quiv2.VData = f(1:4,k);
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