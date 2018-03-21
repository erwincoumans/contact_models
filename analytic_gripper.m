function x = analytic_gripper(params, ux, uz, t)

m = params.m;
r = params.r;
mu = sum(params.mu(2:3));
mf = params.m_g;
mg = 3*mf;

gap = 0.2*r;
g = 9.81;

t_grasp = sqrt(2*gap*mf/ux);

a = 0.5*(uz - m*g)/(m + mg);
b = m*g/(m + mg)*t_grasp;
c = 0.5*(uz/mg)*t_grasp^2 - 0.5*(uz + m*g)/(m + mg)*t_grasp^2 - r;

t_hit = (-b + sqrt(b^2 - 4*a*c))/(2*a);

vhit = m*g/(m + mg)*t_grasp + (uz - m*g)/(m + mg)*(t_hit - t_grasp);
zhit = m*g/(m + mg)*t_grasp*(t_hit - t_grasp) + 0.5*(uz - m*g)/(m + mg)*(t_hit.^2 - t_grasp^2);

t_stop = vhit*m/(mu*ux) + t_hit;
zstop = zhit + vhit*(t_stop - t_hit) - 0.5*mu*ux/m*(t_stop - t_hit)^2;

idx = (t_hit <= t) & (t < t_stop);

z = m*g/(m + mg)*t_grasp*(t - t_grasp) + 0.5*(uz - m*g)/(m + mg)*(t.^2 - t_grasp^2);
z(t < t_grasp) = 0;
z(idx) = zhit + vhit*(t(idx) - t_hit) - 0.5*mu*ux/m*(t(idx) - t_hit).^2;
z(t_stop <= t) = zstop;

x = [zeros(2, numel(t)); r + z];
end