function plot_sphere(r, q, f, filename)

m = 16;
[x, y, z] = sphere(m-1);

X = r*[x(:) y(:) z(:)];
lims = [-3 3 -3 3 0 4]*r;
lims(6) = 4*r + 0.02;

clf
hf = gcf;
hold on
grid on
patch(lims([1 2 2 1]), lims([3 3 4 4]), lims([5 5 5 5]), 0.8+[0 0 0], 'FaceAlpha', 0.3);
Y = reshape(bsxfun(@plus, q(1:3,1)', X*quat2rotm(q(4:7,1)')'), m, 3*m);
h_sphere = surf(Y(:,1:m), Y(:,m+(1:m)), Y(:,2*m+(1:m)), 'FaceColor', [0 0 1], 'FaceAlpha',0.3);
h_quiv = [];
if (nargin >= 3)
    h_quiv = quiver3(q(1,1), q(2,1), q(3,1) - r,...
        f(2,1), f(3,1), f(1,1), 'r');
    h_quiv.LineWidth = 2;
    h_quiv.AutoScale = 'off';
    h_quiv.MaxHeadSize = 0.5;
end
hold off
axis(lims)
axis equal
view(70,5)

if (nargin < 4)
    filename = '';
end

for k = 1:size(q, 2)
    Y = reshape(bsxfun(@plus, q(1:3,k)', X*quat2rotm(q(4:7,k)')'), m, 3*m);
    h_sphere.XData = Y(:,1:m);
    h_sphere.YData = Y(:,m+(1:m));
    h_sphere.ZData = Y(:,2*m+(1:m));
    
    if ~isempty(h_quiv) && (k > 1)
        h_quiv.XData = q(1,k);
        h_quiv.YData = q(2,k);
        h_quiv.ZData = q(3,k) - r;
        h_quiv.UData = f(2,k);
        h_quiv.VData = f(3,k);
        h_quiv.WData = f(1,k);
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