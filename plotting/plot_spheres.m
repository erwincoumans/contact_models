function plot_spheres(nx, r, q)
n = size(q, 1)/7;
N = size(q, 2);

m = 11;
[x, y, z] = sphere(m-1);

X = r*[x(:) y(:) z(:)];
lims = 3*r*[0 nx 0 nx 0 nx];

clf
hold on
grid on
patch(lims([1 2 2 1]), lims([3 3 4 4]), lims([5 5 5 5]), 0.8+[0 0 0], 'FaceAlpha', 0.3);
surf(lims([1 2 2 1 1; 1 2 2 1 1]), lims([3 3 4 4 3; 3 3 4 4 3]), ...
    lims([5 5 5 5 5; 6 6 6 6 6]), 'FaceColor', 0.8+[0 0 0], 'FaceAlpha', 0.3);

h_sphere = cell(1, n);
for i = 1:n
    Y = reshape(bsxfun(@plus, q(7*(i-1)+(1:3),1)', ...
        X*quat2rotm(q(7*(i-1)+(4:7),1)')'), m, 3*m);
    h_sphere{i} = surf(Y(:,1:m), Y(:,m+(1:m)), Y(:,2*m+(1:m)));
end
hold off
axis(lims)

for k = 1:N
    for i = 1:n
        Y = reshape(bsxfun(@plus, q(7*(i-1)+(1:3),k)', ...
            X*quat2rotm(q(7*(i-1)+(4:7),k)')'), m, 3*m);
        h_sphere{i}.XData = Y(:,1:m);
        h_sphere{i}.YData = Y(:,m+(1:m));
        h_sphere{i}.ZData = Y(:,2*m+(1:m));
    end
    axis(lims)
    pause(0.1)
end

end