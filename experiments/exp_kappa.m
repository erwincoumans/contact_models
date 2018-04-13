%% kappa = 0.1
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

% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_ccp;
[x1, f1] = stepper(params, @step_sphere, x0, u, N);
params.step_fun = @solver_convex;
[x2, f2] = stepper(params, @step_sphere, x0, u, N);

%% kappa = 0.02
params.step_fun = @solver_convex;
[x3, f3] = stepper(params, @step_sphere, x0, u, N);

%% kappa = 0.004
params.step_fun = @solver_convex;
[x4, f4] = stepper(params, @step_sphere, x0, u, N);

% Plotting
plot(time, x1(3,:), '-','Color',[0.9290 0.6940 0.1250])
hold on
plot(time, x2(3,:), '-.','Color',[0.2147 0.0079 0.2560])
plot(time, x3(3,:), '--','Color',[0.4940 0.1840 0.5560])
plot(time, x4(3,:), ':','Color',[0.8033 0.5401 0.8560])
hold off

legend('CCP','\kappa = 0.1','\kappa = 0.02','\kappa = 0.004')
xlabel('Time (sec)')
ylabel('Sphere Height (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';
ylim([0.04 0.22])