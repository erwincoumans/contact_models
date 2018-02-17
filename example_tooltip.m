% Cartesian DOF tool (effectively a particle) and a plane.
clear

% Parameters
h = 0.02;
mu = 0.2;
m = 0.2;
params = struct('h', h, 'mu', mu, 'm', m, 'step_fun', []);

x0 = [0, 0, 0, 0.02, 0, 0]';
N = 51;

%% Simulation
time = 0:h:h*(N-1);
[x1, x2, x3] = deal(repmat(x0, 1, N));

params.step_fun = @solver_lcp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.02 - x1(4,k-1)); 0; 0];
    [x1(:,k), ~] = step_tooltip(params, x1(:,k-1), u);
end

params.step_fun = @solver_ccp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.02 - x2(4,k-1)); 0; 0];
    [x2(:,k), ~] = step_tooltip(params, x2(:,k-1), u);
end

params.step_fun = @solver_convex;
for k = 2:N
    u = [mu*9.81*m + 5*(0.02 - x3(4,k-1)); 0; 0];
    [x3(:,k), ~] = step_tooltip(params, x3(:,k-1), u);
end

%% Plotting
plot(time, x1(3,:), '-')
hold on
plot(time, x2(3,:), '--')
plot(time, x3(3,:), ':')
hold off

legend('LCP','CCP','Convex')
xlabel('Time (sec)')
ylabel('Tool Tip Height (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';