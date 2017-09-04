function x_next = forward_lcp(params, x, u)
% q = [th; x; y; w; v_x; v_y]
h = params.h; % time step (s)
mu = params.mu; % Coloumb friction
m = params.m; % disk mass (kg)
r = params.r; % disk radius (m)
m_p = params.m_p; % pusher point mass (kg)

% Fixed matrices
D = [1 -1];
E = [1; 1];
U = mu;
B = [0 0; 1 0; 0 1];
M = diag([0.5*m*r^2 m_p m_p]);

ctr_disk = [r*cos(x(1)); r*sin(x(1))];
ctr_pusher = x(2:3);

% Contact info
n_hat = (ctr_pusher - ctr_disk)/norm(ctr_pusher - ctr_disk); % normal
t_hat = [-n_hat(2); n_hat(1)]; % tangent
psi = norm(ctr_pusher - ctr_disk) - r; % normal distance
r_vec = ctr_disk + r*n_hat; % contact location
% Contact Jacobians
Gn = [r_vec(2)*n_hat(1) - r_vec(1)*n_hat(2); n_hat];
Gt = [r_vec(2)*t_hat(1) - r_vec(1)*t_hat(2); t_hat]*D;

% Gravity
g_vec = [-m*9.81*ctr_disk(1); 0; 0]; % pretend the pusher has perfect gravity compensation

% Helper variables
Gnt = M\Gn;
Gtt = M\Gt;
kv = x(4:6) + M\(B*u + g_vec)*h;

% Solve LCP with Lemke's algorithm
A = [Gn'*Gnt Gn'*Gtt 0; Gt'*Gnt Gt'*Gtt E; U -E' 0];
b = [Gn'*kv + psi/h; Gt'*kv; 0];
z = lemke(A,b);

% Calculate velocity from contact inpulses
v_next = [Gnt Gtt zeros(3,1)]*z + kv;
x_next = [x(1:3) + h*v_next; v_next];

end