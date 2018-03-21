% Sphere and a plane.
clear

% Parameters
h = 0.01;
mu = 0.3;
m = 0.2;
r = 0.05;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'step_fun', []);

x0 = [0, 0, r+0.15, 1, 0, 0, 0, zeros(1, 6)]';
u = zeros(6, 1);
N = 51;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_lcp;
[x1, f1] = stepper(params, @step_sphere, x0, u, N);
params.step_fun = @solver_blcp;
[x2, f2] = stepper(params, @step_sphere, x0, u, N);
params.step_fun = @solver_ccp;
[x3, f3] = stepper(params, @step_sphere, x0, u, N);
params.step_fun = @solver_convex;
[x4, f4] = stepper(params, @step_sphere, x0, u, N);

x5 = analytic_sphere(params, x0(3), time);

%% Plotting
plot(time, x1(3,:), '-')
hold on
plot(time, x2(3,:), '-.')
plot(time, x3(3,:), '--')
plot(time, x4(3,:), ':')
plot(time, x5(3,:), '-k')
hold off

legend('LCP','BLCP','CCP','Convex','Analytic')
xlabel('Time (sec)')
ylabel('Sphere Height (m)')
a = gca;
for k = 2:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';