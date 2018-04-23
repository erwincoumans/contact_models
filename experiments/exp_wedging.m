% Wedge a box between two planes to illustrate slip.
clear

% Parameters
h = 0.01;
mu = 0.9*ones(8,1);
m = 0.2;
lx = 0.01;
ly = 0.01;
lz = 0.02;
w = 0.021;

params = struct('h', h, 'mu', mu, 'm', m, 'lx', lx, 'ly', ly, 'lz', lz, 'w', w, 'step_fun', []);

x0 = [0, 0, lz-w, 1, 0, 0, 0, zeros(1, 6)]';
u = [2, 0, 0, 0, 0, 0]';
N = 201;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_ncp;
[x1, f1] = stepper(params, @step_box, x0, u, N);
params.step_fun = @solver_blcp;
[x2, f2] = stepper(params, @step_box, x0, u, N);
params.step_fun = @solver_ccp;
[x3, f3] = stepper(params, @step_box, x0, u, N);
params.step_fun = @solver_convex;
[x4, f4] = stepper(params, @step_box, x0, u, N);

%% Plotting
plot(time, x1(1,:), '-')
hold on
plot(time, x2(1,:), '-.')
plot(time, x3(1,:), '--')
plot(time, x4(1,:), ':')
hold off

legend({'NCP','BLCP','CCP','Convex'}, 'Location', 'Southeast')
xlabel('Time (sec)')
ylabel('Box X-Position (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end