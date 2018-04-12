% Cartesian DOF tool (effectively a particle) and a plane.
clear

% Parameters
h = 0.01;
mu = 0.3;
m = 0.2;
params = struct('h', h, 'mu', mu, 'm', m, 'step_fun', []);

x0 = [0, 0, 0, 0.2, 0, 0]';
N = 51;

%% Simulation
time = 0:h:h*(N-1);
[x1, x2, x3, x4] = deal(repmat(x0, 1, N));
[f1, f2, f3, f4] = deal(zeros(3, N));

params.step_fun = @solver_lcp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.2 - x1(4,k-1)); 0; 0];
    [x1(:,k), f1(:,k)] = step_tooltip(params, x1(:,k-1), u);
end

params.step_fun = @solver_blcp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.2 - x2(4,k-1)); 0; 0];
    [x2(:,k), f2(:,k)] = step_tooltip(params, x2(:,k-1), u);
end

params.step_fun = @solver_ccp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.2 - x3(4,k-1)); 0; 0];
    [x3(:,k), f3(:,k)] = step_tooltip(params, x3(:,k-1), u);
end

params.step_fun = @solver_convex;
for k = 2:N
    u = [mu*9.81*m + 5*(0.2 - x4(4,k-1)); 0; 0];
    [x4(:,k), f4(:,k)] = step_tooltip(params, x4(:,k-1), u);
end

x5 = analytic_tooltip(params, x0(4), time);

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
ylabel('Particle Height (m)')
a = gca;
for k = 2:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';