function plot_box(params, q, f, filename)

% Parameters
h = params.h;
lx = params.lx; % box x half-length
ly = params.ly; % box y half-length
lz = params.lz; % box z half-length
w = params.w; % slot half-width

x = lx*[ 1 -1 -1  1  1  1 -1 -1  1  1]';
y = ly*[ 1  1 -1 -1  1  1  1 -1 -1  1]';
z = lz*[-1 -1 -1 -1 -1  1  1  1  1  1]';
X = [x y z];

% Plotting
lims = [-3*lx 3*lx -3*ly 3*ly -1.1*w 1.1*w];
clf
patch(lims([1 2 2 1]), lims([3 3 4 4]),-w*ones(1,4), 0.8+[0 0 0]);
patch(lims([1 2 2 1]), lims([3 3 4 4]), w*ones(1,4), 0.8+[0 0 0]);
hold on
grid on

Y = reshape(bsxfun(@plus, q(1:3,1)', X*quat2rotm(q(4:7,1)')'), 5, 6);
h_box = surf(Y(:,1:2)', Y(:,3:4)', Y(:,5:6)','FaceAlpha',0.3);
h_quiv = {};
if (nargin >= 3)
    h_quiv{1} = quiver3(Y(1:4,1), Y(1:4,3), Y(1:4,5), ...
        f(9:12,1), f(17:20,1), f(1:4,1), 'r');
    h_quiv{2} = quiver3(Y(1:4,2), Y(1:4,4), Y(1:4,6), ...
        -f(13:16,1), f(21:24,1), -f(5:8,1), 'r');
    for i = 1:2
        h_quiv{i}.LineWidth = 2;
        h_quiv{i}.AutoScale = 'off';
    end
end
hold off
axis(lims)
view(12,2)

if (nargin < 4)
    filename = '';
end

for k = 1:size(q,2)
    % Plotting
    Y = reshape(bsxfun(@plus, q(1:3,k)', X*quat2rotm(q(4:7,k)')'), 5, 6);
    h_box.XData = Y(:,1:2)';
    h_box.YData = Y(:,3:4)';
    h_box.ZData = Y(:,5:6)';
    zlim([-w w])
    if ~isempty(h_quiv) && (k > 1)
        h_quiv{1}.XData = Y(1:4,1);
        h_quiv{1}.YData = Y(1:4,3);
        h_quiv{1}.ZData = Y(1:4,5);
        h_quiv{1}.UData = f(9:12,k);
        h_quiv{1}.VData = f(17:20,k);
        h_quiv{1}.WData = f(1:4,k);
        h_quiv{2}.XData = Y(1:4,2);
        h_quiv{2}.YData = Y(1:4,4);
        h_quiv{2}.ZData = Y(1:4,6);
        h_quiv{2}.UData = -f(13:16,k);
        h_quiv{2}.VData = f(21:24,k);
        h_quiv{2}.WData = -f(5:8,k);
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