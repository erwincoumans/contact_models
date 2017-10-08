clear
close all

% Parameters
h = 0.02;
mu = [0.3; 0.3; 0.2];
m = 0.1;
r = 0.5;
m_g = 0.8;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_g', m_g, 'step_fun', @forward_lcp);
params_ddp = params;
params_ddp.step_fun = @forward_convex;

% set up the optimization problem
x0 = [0, r, 0, 1.2*r, -1.2*r, 0, zeros(1,6)]'; % initial state
N  = 31; % horizon
rng(0);
u0 = 0.1*randn(3,N-1); % initial controls
u0(:,end) = 0;

time = 0:h:h*(N-1);

x = zeros(size(x0,1), N);
x(:,1) = x0;
[x_plan, u_plan] = deal(cell(1, N-1));
for k = 1:N-1
    fprintf('Step %d\n', k)
    [x_plan{k}, u_plan{k}] = ddp_contact(params_ddp, x(:,k), u0);
    x(:,k+1) = gripper_step(params, x(:,k), u_plan{k}(:,1));
    u0(:,1:end-1) = u_plan{k}(:,2:end); % warm start
end
%%
figure
hold on
z1 = ones(size(time));
for k = 1:N-1
    plot3(time+(k-1)*h, x_plan{k}(2,:), k*z1, 'Color', [(N-k) (N-k) N]/N)
end
plot3(time, x(2,:), 1:N, ':r')

grid on
xlabel('Time (Seconds)')
ylabel('Disk Y-Position')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';
%%
figure
gripper_plot(params, x);


% rep = 4;
% 
% uu = repmat(u,1,1,rep);
% uu = reshape(reshape(permute(uu,[2,1,3]),[],3*rep,1)',3,[]);
% 
% t1 = linspace(0,1,size(u,2));
% t2 = linspace(0,1,rep*size(u,2));
% uu = spline(t1,u,t2);
% 
% params.h = params.h/rep;
% [x_run, f] = gripper_sim(params, x(:,1), uu);
% gripper_plot(params, x_run, f)