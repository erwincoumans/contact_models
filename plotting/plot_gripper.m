function plot_gripper(params, q, x, filename)
% Input:
%   params - system parameters
%   q - pose trajectory
%   x - contact impulses
%   filename - save animation as "filename" (optional)

% Params
h = params.h;
r = params.r; % sphere radius

% Sphere mesh
m = 16;
[xs, ys, zs] = sphere(m-1);
X = r*[xs(:) ys(:) zs(:)];

% Finger mesh
l = 0.005;
xf = 0.010*[ 1 -1 -1  1  1;  1 -1 -1  1  1]';
yf = l*[ 1  1 -1 -1  1;  1  1 -1 -1  1]';
zf = 0.150*[ 0  0  0  0  0;  1  1  1  1  1]';

% Gripper (palm) mesh
xg = 0.050*[ zeros(1,5);  1 -1 -1  1  1;  1 -1 -1  1  1; zeros(1,5)]';
yg = 0.075*[ zeros(1,5);  1  1 -1 -1  1;  1  1 -1 -1  1; zeros(1,5)]';
zg = 2*l*[ zeros(2,5); ones(2,5)]';

% Wrist mesh
[xw, yw, zw] = cylinder(0.025,15);
zw(2,:) = 4*r + 2*l;

% Plot limits
lims = [-3 3 -3 3 0 4]*r;
lims(6) = 4*r + 2*l;
% Plot setup
clf; hf = gcf; hold on; grid on

% Static objects (plane)
patch(lims([1 2 2 1]), lims([3 3 4 4]), lims([5 5 5 5]), 0.8+[0 0 0]);
% Dynamics objects (sphere, fingers, gripper, wrist)
Y = reshape(bsxfun(@plus, q(1:3,1)', X*quat2rotm(q(4:7,1)')'), m, 3*m);
h_sphere = surf(Y(:,1:m), Y(:,m+(1:m)), Y(:,2*m+(1:m)), 'FaceColor', [0 0 1], 'FaceAlpha',0.3);
h_f1 = surf(xf, yf + q(8,1) + l, zf + q(12,1), 'FaceColor', 0.3+[0 0 0], 'FaceAlpha', 1);
h_f2 = surf(xf, yf + q(9,1) - l, zf + q(12,1), 'FaceColor', 0.3+[0 0 0], 'FaceAlpha', 1);
h_g = surf(xg + q(10,1), yg + q(11,1), zg + q(12,1) + 0.15, 'FaceColor', 0.3+[0 0 0], 'FaceAlpha', 1);
h_w = surf(xw, yw, zw, 'FaceColor', 0.3+[0 0 0], 'FaceAlpha', 1);
% Impluse vectors
if (nargin >= 3)
    h_quiv = quiver3(q(1,1) + [0 0 0], q(2,1) + [r -r 0], q(3,1) + [0 0 -r],...
        -x(10:12,1)', [-x(2,1) x(3,1) x(8,1)], [x(6,1) -x(7,1) x(4,1)],...
        'r', 'LineWidth', 2, 'AutoScale', 'off');
end

% More setup
hold off; axis equal; axis(lims)
view(70,5)

for k = 1:size(q,2)
    % Sphere
    Y = reshape(bsxfun(@plus, q(1:3,k)', X*quat2rotm(q(4:7,k)')'), m, 3*m);
    h_sphere.XData = Y(:,1:m);
    h_sphere.YData = Y(:,m+(1:m));
    h_sphere.ZData = Y(:,2*m+(1:m));
    % Fingers
    h_f1.YData = yf + q(8,k) + l;
    h_f1.ZData = zf + q(12,k);
    h_f2.YData = yf + q(9,k) - l;
    h_f2.ZData = zf + q(12,k);
    % Gripper (palm)
    h_g.XData = xg + q(10,k);
    h_g.YData = yg + q(11,k);
    h_g.ZData = zg + q(12,k) + 0.15;
    % Wrist
    h_w.ZData(1,:) = zw(1,:) + q(12,k) + 0.15 + 2*l;
    
    % Impulse vectors
    if (nargin >= 3)
        h_quiv.XData = q(1,k) + [0 0 0];
        h_quiv.YData = q(2,k) + [r -r 0];
        h_quiv.ZData = q(3,k) + [0 0 -r];
        h_quiv.UData = -x(10:12,k)';
        h_quiv.VData = [-x(2,k) x(3,k) x(8,k)];
        h_quiv.WData = [x(6,k) -x(7,k) x(4,k)];
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
