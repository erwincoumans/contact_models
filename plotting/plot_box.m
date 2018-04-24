function plot_box(params, q, x, filename)
% Input:
%   params - system parameters
%   q - pose trajectory
%   x - contact impulses
%   filename - save animation as "filename" (optional)

% Parameters
h = params.h;
lx = params.lx; % box x half-length
ly = params.ly; % box y half-length
lz = params.lz; % box z half-length
w = params.w; % slot half-width

% Box mesh
xb = lx*[ 1 -1 -1  1  1  1 -1 -1  1  1]';
yb = ly*[ 1  1 -1 -1  1  1  1 -1 -1  1]';
zb = lz*[-1 -1 -1 -1 -1  1  1  1  1  1]';
X = [xb yb zb];

% Plot limits
lims = [-3*lx 3*lx -3*ly 3*ly -1.1*w 1.1*w];
% Plot setup
clf; hf = gcf; hold on; grid on

% Static objects (planes)
patch(lims([1 2 2 1]), lims([3 3 4 4]),-w*ones(1,4), 0.8+[0 0 0]);
patch(lims([1 2 2 1]), lims([3 3 4 4]), w*ones(1,4), 0.8+[0 0 0]);
% Dynamics objects (box)
Y = reshape(bsxfun(@plus, q(1:3,1)', X*quat2rotm(q(4:7,1)')'), 5, 6);
h_box = surf(Y(:,1:2)', Y(:,3:4)', Y(:,5:6)', 'FaceColor', [0 0 1], 'FaceAlpha', 0.3);
% Impluse vectors
if (nargin >= 3)
    h_quiv{1} = quiver3(Y(1:4,1), Y(1:4,3), Y(1:4,5), ...
        x(9:12,1), x(17:20,1), x(1:4,1),...
        'r', 'LineWidth', 2, 'AutoScale', 'off');
    h_quiv{2} = quiver3(Y(1:4,2), Y(1:4,4), Y(1:4,6), ...
        -x(13:16,1), x(21:24,1), -x(5:8,1),...
        'r', 'LineWidth', 2, 'AutoScale', 'off');
end

% More setup
hold off; axis equal; axis(lims)
view(12,2)

for k = 1:size(q,2)
    % Box
    Y = reshape(bsxfun(@plus, q(1:3,k)', X*quat2rotm(q(4:7,k)')'), 5, 6);
    h_box.XData = Y(:,1:2)';
    h_box.YData = Y(:,3:4)';
    h_box.ZData = Y(:,5:6)';
    
    % Impulse vectors
    if (nargin >= 3)
        h_quiv{1}.XData = Y(1:4,1);
        h_quiv{1}.YData = Y(1:4,3);
        h_quiv{1}.ZData = Y(1:4,5);
        h_quiv{1}.UData = x(9:12,k);
        h_quiv{1}.VData = x(17:20,k);
        h_quiv{1}.WData = x(1:4,k);
        h_quiv{2}.XData = Y(1:4,2);
        h_quiv{2}.YData = Y(1:4,4);
        h_quiv{2}.ZData = Y(1:4,6);
        h_quiv{2}.UData = -x(13:16,k);
        h_quiv{2}.VData = x(21:24,k);
        h_quiv{2}.WData = -x(5:8,k);
    end
    
    axis(lims)
    
    % GIF Animation
    if (nargin >= 4)
        frame = getframe(hf);
        im = frame2im(frame);
        [im_inds, color_map] = rgb2ind(im, 256);
        if (k == 1)
            imwrite(im_inds, color_map, filename, 'gif', ...
                'Loopcount', Inf, 'DelayTime', 10*h);
        else
            imwrite(im_inds, color_map, filename, 'gif', ...
                'WriteMode', 'append', 'DelayTime', 10*h);
        end
    else
        pause(10*h);
    end
end
end