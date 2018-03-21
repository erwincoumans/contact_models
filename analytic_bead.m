function x = analytic_bead(params, u, t)

m = params.m;
mu = params.mu(1);

g = 9.81;

x = zeros(6, numel(t));
x(4,:) = (u/m - mu*g)*t;
x(1,:) = 0.5*(u/m - mu*g)*t.^2;

end