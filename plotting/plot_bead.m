function plot_bead(r, w, q, f, filename)

m = 16;
[x, y, z] = sphere(m-1);
x = r*x;
y = r*y;
z = r*z;

% Plotting
lims = [-2*r 2*r -2*r 2*r -1.1*w 1.1*w];
clf
hf = gcf;
patch(lims([1 2 2 1]), lims([3 3 4 4]),-w*ones(1,4), 0.8+[0 0 0]);
patch(lims([1 2 2 1]), lims([3 3 4 4]), w*ones(1,4), 0.8+[0 0 0]);
hold on
grid on

h_sphere = surf(x + q(1,1), y + q(2,1), z + q(3,1), 'FaceColor', [0 0 1], 'FaceAlpha',0.3);
h_quiv = {};
if (nargin >= 3)
    h_quiv{1} = quiver3(q(1,1), q(2,1), q(3,1) - r, f(3,1), f(5,1), f(1,1), 'r');
    h_quiv{2} = quiver3(q(1,1), q(2,1), q(3,1) + r,-f(4,1), f(6,1),-f(2,1), 'r');
    h_quiv{1}.LineWidth = 2;
    h_quiv{1}.AutoScale = 'off';
    h_quiv{2}.LineWidth = 2;
    h_quiv{2}.AutoScale = 'off';
end
hold off
axis equal
axis(lims)
view(12,2)

if (nargin < 5)
    filename = '';
end

for k = 1:size(q,2)
    % Plotting
    h_sphere.XData = x + q(1,k);
    h_sphere.YData = y + q(2,k);
    h_sphere.ZData = z + q(3,k);
    if ~isempty(h_quiv) && (k > 1)
        h_quiv{1}.XData = q(1,k);
        h_quiv{1}.YData = q(2,k);
        h_quiv{1}.ZData = q(3,k) - r;
        h_quiv{1}.UData = f(3,k);
        h_quiv{1}.VData = f(5,k);
        h_quiv{1}.WData = f(1,k);
        h_quiv{2}.XData = q(1,k);
        h_quiv{2}.YData = q(2,k);
        h_quiv{2}.ZData = q(3,k) + r;
        h_quiv{2}.UData =-f(4,k);
        h_quiv{2}.VData = f(6,k);
        h_quiv{2}.WData =-f(2,k);
    end
    axis(lims)
    
    if ~isempty(filename)
        frame = getframe(hf);
        im = frame2im(frame);
        [im_inds, color_map] = rgb2ind(im, 256);
        if (k == 1)
            imwrite(im_inds, color_map, filename, 'gif', 'Loopcount', Inf, 'DelayTime', 0.1);
        else
            imwrite(im_inds, color_map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
        end
    else
        pause(0.1);
    end
end
end