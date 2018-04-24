% Slide a particle at a constant velocity to measure sheath effects.
clear

% Parameters
h = 0.01;
mu = 0.3;
m = 0.2;
params = struct('h', h, 'mu', mu, 'm', m, 'step_fun', []);

st0 = [0, 0, 0, 0.2, 0, 0]';
N = 51;

%% Simulation
time = 0:h:h*(N-1);
[st1, st2, st3, st4] = deal(repmat(st0, 1, N));
[x1, x2, x3, x4] = deal(zeros(3, N));

params.step_fun = @solver_ncp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.2 - st1(4,k-1)); 0; 0];
    [st1(:,k), x1(:,k)] = step_particle(params, st1(:,k-1), u);
end

params.step_fun = @solver_blcp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.2 - st2(4,k-1)); 0; 0];
    [st2(:,k), x2(:,k)] = step_particle(params, st2(:,k-1), u);
end

params.step_fun = @solver_ccp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.2 - st3(4,k-1)); 0; 0];
    [st3(:,k), x3(:,k)] = step_particle(params, st3(:,k-1), u);
end

params.step_fun = @solver_qp;
for k = 2:N
    u = [mu*9.81*m + 5*(0.2 - st4(4,k-1)); 0; 0];
    [st4(:,k), x4(:,k)] = step_particle(params, st4(:,k-1), u);
end

%% Plotting
plot(time, st1(3,:), '-')
hold on
plot(time, st2(3,:), '-.')
plot(time, st3(3,:), '--')
plot(time, st4(3,:), ':')
hold off

legend({'NCP','BLCP','CCP','QP'}, 'Location', 'Northeast')
xlabel('Time (sec)')
ylabel('Particle Height (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end