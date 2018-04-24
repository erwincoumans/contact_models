% Slide a particle along a plane to illustrate friction cone approximations.
clear

% Parameters
h = 0.01;
mu = 0.3;
m = 0.2;
params = struct('h', h, 'mu', mu, 'm', m, 'step_fun', []);

st0 = zeros(6,1);
u = [-1, 0.8, 0]';
N = 51;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_ncp;
[st1, x1] = stepper(params, @step_particle, st0, u, N);
params.step_fun = @solver_blcp;
[st2, x2] = stepper(params, @step_particle, st0, u, N);
params.step_fun = @solver_ccp;
[st3, x3] = stepper(params, @step_particle, st0, u, N);
params.step_fun = @solver_lcp;
[st4, x4] = stepper(params, @step_particle, st0, u, N);

%% Plotting
plot(st1(1,:), st1(2,:), '-')
hold on
plot(st2(1,:), st2(2,:), '-.')
plot(st3(1,:), st3(2,:), '--')
plot(st4(1,:), st4(2,:), ':')
hold off
axis equal

legend({'NCP','BLCP','CCP','LCP'}, 'Location', 'Northeast')
xlabel('Particle X-Position (m)')
ylabel('Particle Y-Position (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end