function [st, x] = step_box(params, st, u)
% st = [x, y, z, q0, q1, q2, q3, ...]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % box mass
lx = params.lx; % box x half-length
ly = params.ly; % box y half-length
lz = params.lz; % box z half-length
w = params.w; % slot half-width
step_fun = params.step_fun;

M = diag(m*[1 1 1 (ly^2+lz^2)/3 (lx^2+lz^2)/3 (lx^2+ly^2)/3]);

% Extract pose and velocity
q = st(1:7);
v = st(8:13);

% Gravitational, external, and other forces
omega = v(4:6);
I = M(4:6,4:6);
Fext = [0; 0; -9.81*m; -cross(omega, I*omega)] + u;

% Positions of box corners
R = quat2rotm(q(4:7)');
rs = R*[ lx -lx -lx  lx  lx -lx -lx  lx
         ly  ly -ly -ly  ly  ly -ly -ly
        -lz -lz -lz -lz  lz  lz  lz  lz];
ps = repmat(q(1:3), 1, 8) + rs;

% Contact gap distances
psi =  [ps(3,1:4) + w, w - ps(3,5:8)]';

% Jacobian for contacts
J = [ 0  0  0  0  0  0  0  0  1  1  1  1 -1 -1 -1 -1  0  0  0  0  0  0  0  0
      0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  1  1  1  1  1  1  1  1
      1  1  1  1 -1 -1 -1 -1  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0
      zeros(3,24)];
for i = 0:23
  J(4:6,i+1)  = R'*cross(rs(:,mod(i,8)+1), J(1:3,i+1));
end
J = J';

 % All contacts active at all times 
[v_next, x]  = step_fun(v, Fext, M, J, mu, psi, h);
q_next = int_body(q, v_next, h);

st = [q_next; v_next];
end