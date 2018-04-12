function [st, x] = step_bead(params, st, u)
% st = [x, y, z, ...]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % sphere mass
r = params.r; % sphere radius
w = params.w; % slot half-width
step_fun = params.step_fun;

M = diag([m m m]);

% Extract pose and velocity
q = st(1:3);
v = st(4:6);

% Gravitational, external, and other forces
Fext = [0; 0; -9.81*m] + u;

% Contact normal distances (gaps)
psi = [w + (q(3) - r)
       w - (q(3) + r)];

% Jacobian for contacts
J = [ 0  0  1   % normal 1
      0  0 -1   % normal 2
      1  0  0   % tangent 1
     -1  0  0   % tangent 2
      0  1  0   % other 1
      0  1  0]; % other 2

[v_next, x]  = step_fun(v, Fext, M, J, mu, psi, h);
q_next = q + h*v_next;

st = [q_next; v_next];
end