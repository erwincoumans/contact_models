function [st, x] = step_gripper(params, st, u)
% st = [x, y, z, q0, q1, q2, q3, y_f1, y_f2, x_g, y_g, z_g ...]

% System parameters
h = params.h;
mu = params.mu;
m = params.m; % sphere mass
r = params.r; % sphere radius
m_g = params.m_g; % gripper finger mass
step_fun = params.step_fun;

M = diag([m*[1 1 1 (2/5)*r^2 (2/5)*r^2 (2/5)*r^2] m_g*[1 1 3 3 3]]);

% Extract pose and velocity
q = st(1:12);
v = st(13:23);

% Gravitational, external, and other forces
omega = v(4:6);
I = M(4:6,4:6);
Fext = [0; 0; -9.81*m; -cross(omega, I*omega); u];

% Contact gap distances
psi = [r - q(12)
       q(8) - q(2) - r
       q(2) - q(9) - r
       q(3) - r];

% Jacobian for contacts
J = [ zeros(1,8)              0  0 -1   % gripper lift height   (normal)
      0 -1  0  0  0  0  1  0  0  0  0   % finger1-disk          (normal)
      0  1  0  0  0  0  0 -1  0  0  0   % finger2-disk          (normal)
      0  0  1  0  0  0  0  0  0  0  0   % disk-floor            (normal)
      zeros(1,8)              0  1  0   % gripper lift height  (tangent)
      0  0  1  r  0  0  0  0  0  0 -1   % finger1-disk         (tangent)
      0  0 -1  r  0  0  0  0  0  0  1   % finger2-disk         (tangent)
      0  1  0  r  0  0  0  0  0  0  0   % disk-floor           (tangent)
      zeros(1,8)             -1  0  0   % gripper lift height    (other)
     -1  0  0  0  0 -r  0  0  0  0  0   % finger1-disk           (other)
     -1  0  0  0  r  0  0  0  0  0  0   % finger2-disk           (other)
     -1  0  0  0  0  r  0  0  0  0  0]; % disk-floor             (other)
R = quat2rotm(q(4:7)');
J(:,4:6) = J(:,4:6)*R;

% Solve contact dynamics
[v_next, x]  = step_fun(v, Fext, M, J, mu, psi, h);

q_next = q;
q_next(1:7) = int_body(q(1:7), v_next(1:6), h);
q_next(8:12) = q(8:12) + h*v_next(7:11);

st = [q_next; v_next];
end