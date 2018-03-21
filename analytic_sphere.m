function x = analytic_sphere(params, z0, t)

r = params.r;

g = 9.81;
t_impact = sqrt(2*(z0 - r)/g);
mask = (t >= t_impact);

x = zeros(13, numel(t));
x(4,:) = 1;

x(10,:) = -g*t;
x(10,mask) = 0;

x(3,:) = z0 - 0.5*g*t.^2;
x(3,mask) = r;

end