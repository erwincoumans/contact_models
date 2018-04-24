function plot_bead(params, q, x, filename)
% Input:
%   params - system parameters
%   q - pose trajectory
%   x - contact impulses
%   filename - save animation as "filename" (optional)

% Parameters
h = params.h;
r = params.r; % sphere radius
w = params.w; % slot half-width

% Sphere mesh
m = 16;
[xs, ys, zs] = sphere(m-1);
xs = r*xs;
ys = r*ys;
zs = r*zs;

% Plot limits
lims = [-1*r 2*r -2*r 2*r -1.1*w 1.1*w];
% Plot setup
clf; hf = gcf; hold on; grid on

% Static objects (planes)
patch(lims([1 2 2 1]), lims([3 3 4 4]),-w*ones(1,4), 0.8+[0 0 0]);
patch(lims([1 2 2 1]), lims([3 3 4 4]), w*ones(1,4), 0.8+[0 0 0]);
% Dynamics objects (sphere)
h_sphere = surf(xs + q(1,1), ys + q(2,1), zs + q(3,1), 'FaceColor', [0 0 1], 'FaceAlpha',0.3);
% Impluse vectors
if (nargin >= 3)
    h_quiv{1} = quiver3(q(1,1), q(2,1), q(3,1) - r,...
         x(3,1), x(5,1), x(1,1), 'r', 'LineWidth', 2, 'AutoScale', 'off');
    h_quiv{2} = quiver3(q(1,1), q(2,1), q(3,1) + r,...
        -x(4,1), x(6,1),-x(2,1), 'r', 'LineWidth', 2, 'AutoScale', 'off');
end

% More setup
hold off; axis equal; axis(lims)
view(0,0)

for k = 1:size(q,2)
    % Sphere
    h_sphere.XData = xs + q(1,k);
    h_sphere.YData = ys + q(2,k);
    h_sphere.ZData = zs + q(3,k);
    
    % Impulse vectors
    if (nargin >= 3)
        h_quiv{1}.XData = q(1,k);
        h_quiv{1}.YData = q(2,k);
        h_quiv{1}.ZData = q(3,k) - r;
        h_quiv{1}.UData = x(3,k);
        h_quiv{1}.VData = x(5,k);
        h_quiv{1}.WData = x(1,k);
        h_quiv{2}.XData = q(1,k);
        h_quiv{2}.YData = q(2,k);
        h_quiv{2}.ZData = q(3,k) + r;
        h_quiv{2}.UData =-x(4,k);
        h_quiv{2}.VData = x(6,k);
        h_quiv{2}.WData =-x(2,k);
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