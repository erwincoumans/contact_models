% Sphere and a plane.
clear

% Parameters
h = 0.02;
mu = 0.2;
m = 0.2;
r = 0.05;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'step_fun', []);

x0 = [0, 0, 0.025, 1, 0, 0, 0, zeros(1, 6)]';
u = zeros(6, 1);
N = 11;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_lcp;
[x1, ~] = stepper(params, @step_sphere, x0, u, N);
params.step_fun = @solver_ccp;
[x2, ~] = stepper(params, @step_sphere, x0, u, N);
params.step_fun = @solver_convex;
[x3, ~] = stepper(params, @step_sphere, x0, u, N);

%% Plotting
plot(time, x1(3,:), '-')
hold on
plot(time, x2(3,:), '--')
plot(time, x3(3,:), ':')
hold off

legend('LCP','CCP','Convex')
xlabel('Time (sec)')
ylabel('Sphere Height (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';