function plot_spheres(params, q, ~, filename)
% Input:
%   params - system parameters
%   q - pose trajectory
%   x - contact impulses
%   filename - save animation as "filename" (optional)

% Parameters
h = params.h;
r = params.r;
nx = params.nx;
nz = params.nz;

n = size(q, 1)/7;
N = size(q, 2);

% Sphere mesh
m = 11;
[xs, ys, zs] = sphere(m-1);
X = r*[xs(:) ys(:) zs(:)];

% Plot limits
lims = 3*r*[0 nx 0 nx 0 nz];
% Plot setup
clf; hf = gcf; hold on; grid on

% Static objects (box)
patch(lims([1 2 2 1]), lims([3 3 4 4]), lims([5 5 5 5]), 0.8+[0 0 0], 'FaceAlpha', 0.3);
surf(lims([1 2 2 1 1; 1 2 2 1 1]), lims([3 3 4 4 3; 3 3 4 4 3]), ...
    lims([5 5 5 5 5; 6 6 6 6 6]), 'FaceColor', 0.8+[0 0 0], 'FaceAlpha', 0.3);
% Dynamics objects (spheres)
h_sphere = cell(1, n);
for i = 1:n
    Y = reshape(bsxfun(@plus, q(7*(i-1)+(1:3),1)', ...
        X*quat2rotm(q(7*(i-1)+(4:7),1)')'), m, 3*m);
    h_sphere{i} = surf(Y(:,1:m), Y(:,m+(1:m)), Y(:,2*m+(1:m)));
end

% More setup
hold off; axis equal; axis(lims)

for k = 1:N
    % Spheres
    for i = 1:n
        Y = reshape(bsxfun(@plus, q(7*(i-1)+(1:3),k)', ...
            X*quat2rotm(q(7*(i-1)+(4:7),k)')'), m, 3*m);
        h_sphere{i}.XData = Y(:,1:m);
        h_sphere{i}.YData = Y(:,m+(1:m));
        h_sphere{i}.ZData = Y(:,2*m+(1:m));
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