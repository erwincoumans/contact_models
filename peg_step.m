function [y, f] = peg_step(params, x, u)
% x = [q; v]
% q = [x_disk, y_disk, th_disk, x1_grip, x2_grip, y_grip]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % peg mass
r = params.r; % peg radius
s = params.s; % peg length (half)
w = params.w; % hole radius
step_fun = params.step_fun;

M = diag([m m m*(r^2+s^2)/3]);

% Extract pose and velocity
n = size(x,1);
q = x(1:n/2);
v = x(n/2+1:end);

sth = sin(q(3));
cth = cos(q(3));

% Gravitational, external, and other forces
Fext = [0; -9.81*m; 0] + u;

% Contact normal distances (gaps)
psi = [w - (q(1) + s*sth + r*cth)
       (q(1) + s*sth - r*cth) + w
       (w - q(1))*cth + (0 - q(2))*sth - r
       (q(1) + w)*cth + (q(2) - 0)*sth - r];

% Jacobian for contacts
J = [-1    0    r*sth - s*cth % normal 1
      1    0    r*sth + s*cth % normal 2
     -cth  sth  0 % TODO      % normal 3
      cth  sth  0 % TODO      % normal 4
      0   -1   -r*cth - s*sth % tangent 1
      0    1   -r*cth + s*sth % tangent 2
      sth -cth -r             % tangent 3
     -sth  cth -r];           % tangent 4

 % All contacts active at all times
 
[q_next, v_next, f] = step_fun(h, M, q, v, Fext, mu, psi, J);

y = [q_next; v_next];
end