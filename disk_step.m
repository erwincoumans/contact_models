function [y, f] = disk_step(params, x, u)
% x = [x_pos, y_pos, theta, x_vel, y_vel, omega]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % disk mass
r = params.r; % disk radius
w = params.w; % hole radius
step_fun = params.step_fun;

M = diag([m m]);

% Extract pose and velocity
n = size(x,1);
q = x(1:n/2);
v = x(n/2+1:end);

% Gravitational, external, and other forces
Fext = [0; -9.81*m] + u;

% Contact normal distances (gaps)
psi = [w - (q(2) + r)
       w + (q(2) - r)];

% Jacobian for contacts
J = [ 0 -1
      0  1
     -1  0
      1  0];

[q_next, v_next, f] = step_fun(h, M, q, v, Fext, mu, psi, J);

y = [q_next; v_next];
end