function [y, f] = peg_step(params, x, u)
% x = [q; v]
% q = [x_disk, y_disk, th_disk, x1_grip, x2_grip, y_grip]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % peg mass
r = params.r; % peg radius
l = params.l; % peg length (half)
w = params.w; % hole radius
step_fun = params.step_fun;

M = diag([m m m*(r^2+l^2)/3]);

% Extract pose and velocity
n = size(x,1);
q = x(1:n/2);
v = x(n/2+1:end);

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
 
[q_next, v_next, f] = step_fun(h, M, q, v, Fext, mu, psi, J);

y = [q_next; v_next];
end