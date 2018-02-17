function [st, x] = step_sphere(params, st, u)
% st = [x, y, z, q0, q1, q2, q3, ...]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % sphere mass
r = params.r; % sphere radius
step_fun = params.step_fun;

M = diag(m*[1 1 1 (2/5)*r^2 (2/5)*r^2 (2/5)*r^2]);

% Extract pose and velocity
q = st(1:7);
v = st(8:13);

% Gravitational, external, and other forces
w = v(4:6);
I = M(4:6,4:6);
Fext = [0; 0; -9.81*m; -cross(w, I*w)] + u;

% Contact normal distances (gaps)
psi = q(3);

% Jacobian for contacts
J = [ 0  1  0; 0  0  1; 1  0  0; R'*[0 0 r; 0 -r 0; 0 0 0]]';

% Step without contact impulses
v_next = (v + M\Fext*h);
q_next = int_body(q, v_next, h);
x = [0; 0; 0];

% Identify active contacts
if (q_next(3) < 0.01)
    [v_next, x]  = step_fun(v, Fext, M, J, mu, psi, h);
    q_next = int_body(q, v_next, h);
end

st = [q_next; v_next];
end