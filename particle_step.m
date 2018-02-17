function [st, x] = particle_step(params, st, u)
% st = [x_pos, y_pos, x_vel, y_vel]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % particle mass
step_fun = params.step_fun;

M = diag([m m]);

% Extract pose and velocity
q = st(1:2);
v = st(3:4);

% Gravitational, external, and other forces
Fext = [0; -9.81*m] + u;

% Contact normal distances (gaps)
psi = q(2);

% Jacobian for contacts
J = [0  1; 1 0];

% Step without contact impulses
v_next = (v + M\Fext*h);
q_next = q + h*v_next;
x = [0; 0];

% Identify active contacts
if (q_next(2) < 0.01)
    [q_next, v_next, x] = step_fun(q, v, Fext, M, J, mu, psi, h);
end

st = [q_next; v_next];
end