% Grasp using constant control inputs.
clear

% Parameters
h = 0.02;
mu = 0.2;
m = 0.2;
params = struct('h', h, 'mu', mu, 'm', m, 'step_fun', []);

x0 = [0, 0.025, 0, 0]';
u = [0, 0]';
N = 11;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_lcp;
[x1, ~] = stepper(params, @step_particle, x0, u, N);
params.step_fun = @solver_ccp;
[x2, ~] = stepper(params, @step_particle, x0, u, N);
params.step_fun = @solver_convex;
[x3, ~] = stepper(params, @step_particle, x0, u, N);

%% Plotting
plot(time, x1(2,:), '-')
hold on
plot(time, x2(2,:), '--')
plot(time, x3(2,:), ':')
hold off

legend('LCP','CCP','Convex')
xlabel('Time (sec)')
ylabel('Particle Y-Position (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';