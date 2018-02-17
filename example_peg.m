% Grasp using constant control inputs.
clear

% Parameters
h = 0.02;
mu = 0.9*ones(4,1);
m = 0.2;
r = 0.02;
l = 0.01;
w = 0.021;

params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'l', l, ...
    'w', w, 'step_fun', @forward_lcp);

x0 = [0, r-w, 0, 0, 0, 0]';
u = [2, 0, 0]';
N = 101;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @forward_lcp;
[x1, f1] = stepper(params, @peg_step, x0, u, N);
params.step_fun = @forward_ccp;
[x2, f2] = stepper(params, @peg_step, x0, u, N);
params.step_fun = @forward_convex;
[x3, f3] = stepper(params, @peg_step, x0, u, N);

%% Plotting
plot(time, x1(1,:), '-')
hold on
plot(time, x2(1,:), '--')
plot(time, x3(1,:), ':')
hold off

legend('LCP','CCP','Convex')
xlabel('Time (sec)')
ylabel('Peg X-Position (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';