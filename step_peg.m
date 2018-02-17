function [st, x] = step_peg(params, st, u)
% st = [x_pos, y_pos, theta, x_vel, y_vel, omega]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % peg mass
r = params.r; % peg radius
l = params.l; % peg half-length
w = params.w; % slot half-width
step_fun = params.step_fun;

M = diag([m m m*(r^2+l^2)/3]);

% Extract pose and velocity
q = st(1:3);
v = st(4:6);

sth = sin(q(3));
cth = cos(q(3));

% Gravitational, external, and other forces
Fext = [0; -9.81*m; 0] + u;

% Contact normal distances (gaps)
psi = [w - (q(2) + r*cth - l*sth)
       w - (q(2) + r*cth + l*sth)
       w + (q(2) - r*cth + l*sth)
       w + (q(2) - r*cth - l*sth)];

% Jacobian for contacts
J = [ 0 -1  l*cth + r*sth   % normal 1
      0 -1 -l*cth + r*sth   % normal 2
      0  1  l*cth - r*sth   % normal 3
      0  1 -l*cth - r*sth   % normal 4
     -1  0  r*cth - l*sth   % tangent 1
     -1  0  r*cth + l*sth   % tangent 2
      1  0  r*cth - l*sth   % tangent 3
      1  0  r*cth + l*sth]; % tangent 4

 % All contacts active at all times
 
[q_next, v_next, x] = step_fun(q, v, Fext, M, J, mu, psi, h);

st = [q_next; v_next];
end