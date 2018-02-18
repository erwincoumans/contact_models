% Cartesian DOF tool (effectively a particle) and a plane.
clear

% Parameters
h = 0.02;
mu = 0.2;
m = 0.2;
params = struct('h', h, 'mu', mu, 'm', m, 'step_fun', []);

x0 = zeros(6,1);
u = [-1, 0.5, 0]';
N = 11;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_lcp;
[x1, f1] = stepper(params, @step_tooltip, x0, u, N);
params.step_fun = @solver_blcp;
[x2, f2] = stepper(params, @step_tooltip, x0, u, N);
params.step_fun = @solver_ccp;
[x3, f3] = stepper(params, @step_tooltip, x0, u, N);
params.step_fun = @solver_convex;
[x4, f4] = stepper(params, @step_tooltip, x0, u, N);

%% Plotting
plot(x1(1,:), x1(2,:), '-')
hold on
plot(x2(1,:), x2(2,:), '-.')
plot(x3(1,:), x3(2,:), '--')
plot(x4(1,:), x4(2,:), ':')
hold off

legend('LCP','BLCP','CCP','Convex')
xlabel('Tool X-Position (m)')
ylabel('Tool Y-Position (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';