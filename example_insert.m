% Grasp using constant control inputs.
clear

% Parameters
h = 0.02;
mu = 0.2*ones(2,1);
m = 0.2;
r = 0.02;
l = 0.05;
w = 0.021;

params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'l', l, ...
    'w', w, 'step_fun', @forward_lcp);

x0 = [0, r-w, 0, 0]';
u = [1, 0]';
N = 31;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @forward_lcp;
[x1, f1] = stepper(params, @disk_step, x0, u, N);
params.step_fun = @forward_ccp;
[x2, f2] = stepper(params, @disk_step, x0, u, N);
params.step_fun = @forward_convex;
[x3, f3] = stepper(params, @disk_step, x0, u, N);

%% Plotting
plot(time, x1(3,:), '-')
hold on
plot(time, x2(3,:), '--')
plot(time, x3(3,:), ':')
hold off

legend('LCP','CCP','Convex')
xlabel('Time (sec)')
ylabel('Disk X-Velocity (m/s)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';