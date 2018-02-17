% Tune parameters so DDP works.

clear
close all

% Parameters
h = 0.02;
mu = [0.2; 0.2; 0.3; 0.3; 0.2];
m = 0.1;
r = 0.05;
m_g = 0.8;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_g', m_g, 'step_fun', @solver_lcp);
params.fd = 1e-4;
op = struct('plot', 1, 'print', 1);

% set up the optimization problem
x0 = [0, r, 0, 1.2*r, -1.2*r, 0, zeros(1,6)]'; % initial state
T = 30; % horizon
rng(0);
u0 = 0.1*randn(3,T); % initial controls
u0(:,end) = 0;

time = 0:h:h*T;

[x, u] = ddp_grasp(params, op, x0, u0);

plot_gripper(params, x)