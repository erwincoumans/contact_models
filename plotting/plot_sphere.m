function plot_sphere(params, q, x, filename)
% Input:
%   params - system parameters
%   q - pose trajectory
%   x - contact impulses
%   filename - save animation as "filename" (optional)

% Parameters
h = params.h;
r = params.r; % sphere radius

% Sphere mesh
m = 16;
[xs, ys, zs] = sphere(m-1);
X = r*[xs(:) ys(:) zs(:)];

% Plot limits
lims = [-3 3 -3 3 0 4]*r;
lims(6) = 4*r + 0.02;
% Plot setup
clf; hf = gcf; hold on; grid on

% Static objects (plane)
patch(lims([1 2 2 1]), lims([3 3 4 4]), lims([5 5 5 5]), 0.8+[0 0 0], 'FaceAlpha', 0.3);
% Dynamics objects (sphere)
Y = reshape(bsxfun(@plus, q(1:3,1)', X*quat2rotm(q(4:7,1)')'), m, 3*m);
h_sphere = surf(Y(:,1:m), Y(:,m+(1:m)), Y(:,2*m+(1:m)), 'FaceColor', [0 0 1], 'FaceAlpha',0.3);
% Impluse vectors
if (nargin >= 3)
    h_quiv = quiver3(q(1,1), q(2,1), q(3,1) - r,...
        x(2,1), x(3,1), x(1,1), 'r', 'LineWidth', 2, 'AutoScale', 'off');
end

% More setup
hold off; axis equal; axis(lims)
view(70,5)

for k = 1:size(q, 2)
    % Sphere
    Y = reshape(bsxfun(@plus, q(1:3,k)', X*quat2rotm(q(4:7,k)')'), m, 3*m);
    h_sphere.XData = Y(:,1:m);
    h_sphere.YData = Y(:,m+(1:m));
    h_sphere.ZData = Y(:,2*m+(1:m));
    
    % Impulse vectors
    if (nargin >= 3)
        h_quiv.XData = q(1,k);
        h_quiv.YData = q(2,k);
        h_quiv.ZData = q(3,k) - r;
        h_quiv.UData = x(2,k);
        h_quiv.VData = x(3,k);
        h_quiv.WData = x(1,k);
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