function [y, f] = particle_step(params, x, u)
% x = [x_pos, y_pos, x_vel, y_vel]

% System parameters
h = params.h;
mu = params.mu;
m = params.m;
step_fun = params.step_fun;

M = diag([m m]);

% Extract pose and velocity
q = x(1:2);
v = x(3:4);

% Contact normal distances (gaps)
psi = q(2);

% Jacobian for contacts
J = [0  1; 1 0];

v_next = (v + M\u*h);
q_next = q + h*v_next;
f = [0; 0];

if (q_next(2) < 0.01)
    [q_next, v_next, f] = step_fun(h, M, q, v, u, mu, psi, J);
end

y = [q_next; v_next];
end