% Plan grasp with various finite difference sizes.

clear
close all

% Parameters
h = 0.02;
mu = [0.3; 0.3; 0.2];
m = 0.1;
r = 0.05;
m_g = 0.8;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_g', m_g, 'step_fun', @forward_lcp);
params_ddp = params;
params_ddp.fd = 1e-4; % set below
params_ddp.step_fun = @forward_convex;
op = struct('plot', 0, 'print', 1, 'maxIter', 500);

% set up the optimization problem
x0 = [0, r, 0, 1.2*r, -1.2*r, 0, zeros(1,6)]'; % initial state
T = 30; % horizon
rng(0);
u0 = 0.1*randn(3,T); % initial controls
u0(:,end) = 0;

fd = logspace(-1,-8,8);
N = numel(fd);
x = cell(size(fd));
[cst,iter] = deal(NaN(size(fd)));
for k = 1:N
    params_ddp.fd = fd(k);
    [x{k}, ~, cst(k), iter(k)] = ddp_contact(params_ddp, op, x0, u0);
end

%% Planned trajectories
figure
hold on
time = 0:h:h*T;
z1 = ones(size(time));
for k = 1:N
    plot3(time, x{k}(2,:), k*z1, 'Color', [(N-k) 0 k]/N)
end

grid on
xlabel('Time (Seconds)')
ylabel('Disk Y-Position')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';

%% Trajectory costs and # iterations until convergence
figure
semilogx(fd,cst)
xlabel('Finite Difference \Delta')
ylabel('Trajectory Cost')
grid on
a = gca;
a.Children(1).LineWidth = 2;
a.FontSize = 14;
a.FontWeight = 'bold';

figure
semilogx(fd,iter)
xlabel('Finite Difference \Delta')
ylabel('DDP Iterations')
grid on
a = gca;
a.Children(1).LineWidth = 2;
a.FontSize = 14;
a.FontWeight = 'bold';