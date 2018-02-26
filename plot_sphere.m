function plot_sphere(r, q)

m = 16;
[x, y, z] = sphere(m-1);

X = r*[x(:) y(:) z(:)];
lims = [-3 3 -3 3 0 4]*r;
lims(6) = 4*r + 0.02;

clf
hold on
grid on
patch(lims([1 2 2 1]), lims([3 3 4 4]), lims([5 5 5 5]), 0.8+[0 0 0], 'FaceAlpha', 0.3);
Y = reshape(bsxfun(@plus, q(1:3,1)', X*quat2rotm(q(4:7,1)')'), m, 3*m);
h_sphere = surf(Y(:,1:m), Y(:,m+(1:m)), Y(:,2*m+(1:m)), 'FaceColor', [0 0 1], 'FaceAlpha',0.3);
hold off
axis(lims)
axis equal
view(70,5)

for k = 1:size(q, 2)
    Y = reshape(bsxfun(@plus, q(1:3,k)', X*quat2rotm(q(4:7,k)')'), m, 3*m);
    h_sphere.XData = Y(:,1:m);
    h_sphere.YData = Y(:,m+(1:m));
    h_sphere.ZData = Y(:,2*m+(1:m));
    axis(lims)
    pause(0.1)
end

end