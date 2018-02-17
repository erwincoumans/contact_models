function [st, x] = gripper_step(params, st, u)
% st = [x_disk, y_disk, th_disk, x1_grip, x2_grip, y_grip, ...]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % disk mass
r = params.r; % disk radius
m_g = params.m_g; % gripper finger mass
step_fun = params.step_fun;

M = diag([m m 0.5*m*r^2 m_g m_g 2*m_g]);

% Extract pose and velocity
q = st(1:6);
v = st(7:12);

% Gravitational, external, and other forces
Fext = [0; -9.81*m; 0; u];

% Contact normal distances (gaps)
psi = [r - q(6)
       q(6)
       q(4) - (q(1) + r)
       (q(1) - r) - q(5)
       q(2) - r];

% Jacobian for contacts
J = [ 0  0  0  0  0 -1   % gripper lift height   (normal)
      0  0  0  0  0  1   % gripper lower height  (normal)
     -1  0  0  1  0  0   % finger1-disk          (normal)
      1  0  0  0 -1  0   % finger2-disk          (normal)
      0  1  0  0  0  0   % disk-floor            (normal)
      0  0  0  1  1  0   % gripper lift height  (tangent)
      0  0  0 -1 -1  0   % gripper lower height (tangent)
      0 -1 -r  0  0  1   % finger1-disk         (tangent)
      0  1 -r  0  0 -1   % finger2-disk         (tangent)
     -1  0 -r  0  0  0]; % disk-floor           (tangent)

% Step without contact impulses
v_next = (v + M\Fext*h);
q_next = q + h*v_next;
psi2 = [r - q_next(6)
        q_next(6)
        q_next(4) - (q_next(1) + r)
        (q_next(1) - r) - q_next(5)
        q_next(2) - r];

% Identify active contacts
c_active = psi2 < 0.01;
J = J([c_active; c_active],:);
psi = psi(c_active);
mu = mu(c_active);

x = NaN(2*size(c_active,1),1);
if any(c_active)
    % Solve contact dynamics
    [q_next, v_next, x_active] = step_fun(q, v, Fext, M, J, mu, psi, h);
    x([c_active; c_active]) = x_active;
end

st = [q_next; v_next];
end