function [st, x] = disk_step(params, st, u)
% st = [x_pos, y_pos, theta, x_vel, y_vel, omega]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % disk mass
r = params.r; % disk radius
w = params.w; % slot half-width
step_fun = params.step_fun;

M = diag([m m]);

% Extract pose and velocity
q = st(1:2);
v = st(3:4);

% Gravitational, external, and other forces
Fext = [0; -9.81*m] + u;

% Contact normal distances (gaps)
psi = [w - (q(2) + r)
       w + (q(2) - r)];

% Jacobian for contacts
J = [0 -1; 0  1; -1  0; 1  0];

[q_next, v_next, x] = step_fun(q, v, Fext, M, J, mu, psi, h);

st = [q_next; v_next];
end