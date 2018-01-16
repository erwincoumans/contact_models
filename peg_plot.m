function peg_plot(params, x, f, filename)

% Parameters
h = params.h;
r = params.r; % peg radius
s = params.s; % peg length (half)
w = params.w; % hole radius

% Plotting
lims = [-4 4 -4 4]*w;
clf
hf = gcf;
patch(-[4 1 1 4]*w, -[4 4 0 0]*s, 0.8+[0 0 0]);
patch( [4 1 1 4]*w, -[4 4 0 0]*s, 0.8+[0 0 0]);

h_disk = patch(x(1) + [-r r r -r]*cos(x(3)) - [-s -s s s]*sin(x(3)), ...
               x(2) + [-r r r -r]*sin(x(3)) + [-s -s s s]*cos(x(3)), ...
               0.8+[0 0 0]);
h_quiv1 = [];
if (nargin >= 3)
    hold on
    h_quiv1 = quiver([0 0 x(4) x(5)], [x(6)+4*r x(6)+3*r x(2) x(2)],...
        [0 0 f(4) -f(5)], [-f(2) f(3) f(7) -f(8)], 0, 'b');
    h_quiv2 = quiver([x(1)+r x(1)-r x(1)], [x(2) x(2) 0],...
        [-f(4) f(5) -f(9)], [-f(7) f(8) f(6)], 0, 'r');
    hold off
end
axis(lims)

if (nargin < 4)
    filename = '';
end

for k = 1:size(x,2)
    % Plotting
    h_disk.XData = x(1,k) + [-r r r -r]*cos(x(3,k)) - [-s -s s s]*sin(x(3,k));
    h_disk.YData = x(2,k) + [-r r r -r]*sin(x(3,k)) + [-s -s s s]*cos(x(3,k));
    if ~isempty(h_quiv1) && (k <= size(f,2))
        h_quiv1.XData = [0 0 x(4,k) x(5,k)];
        h_quiv1.YData = [x(6,k)+4*r x(6,k)+3*r x(2,k) x(2,k)];
        h_quiv1.UData = [0 0 f(4,k) -f(5,k)];
        h_quiv1.VData = [-f(2,k) f(3,k) f(7,k) -f(8,k)];
        h_quiv2.XData = [x(1,k)+r x(1,k)-r x(1,k)];
        h_quiv2.YData = [x(2,k) x(2,k) 0];
        h_quiv2.UData = [-f(4,k) f(5,k) -f(9,k)];
        h_quiv2.VData = [-f(7,k) f(8,k) f(6,k)];
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