% Grasp using constant control inputs.

% Note: with convex contact model, the gripper opens a bit when it hits
% the high stop. Larger h exacerbates the issue.
clear

% Parameters
h = 0.01;
mu = 0.3*ones(5,1);
m = 0.2;
r = 0.05;
m_g = 2.0;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_g', m_g, 'step_fun', []);

x0 = [0, 0, r, 1, 0, 0, 0, 1.2*r, -1.2*r, 0, 0, 0, zeros(1,11)]';
u = [-4 4 0 0 5]';
N = 101;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_lcp;
[x1, f1] = stepper(params, @step_gripper, x0, u, N);
params.step_fun = @solver_blcp;
[x2, f2] = stepper(params, @step_gripper, x0, u, N);
params.step_fun = @solver_ccp;
[x3, f3] = stepper(params, @step_gripper, x0, u, N);
params.step_fun = @solver_convex;
[x4, f4] = stepper(params, @step_gripper, x0, u, N);

%% Plotting
hs(1) = subplot(2,1,1);
plot(time, x1(3,:), '-')
hold on
plot(time, x2(3,:), '-.')
plot(time, x3(3,:), '--')
plot(time, x4(3,:), ':')
hold off
ylim([0.08, 0.1])
xlim([0 1])
ylabel('Sphere Height (m)')

% Plotting
hs(2) = subplot(2,1,2);
plot(time, x1(8,:)-x1(9,:), '-')
hold on
plot(time, x2(8,:)-x2(9,:), '-.')
plot(time, x3(8,:)-x3(9,:), '--')
plot(time, x4(8,:)-x4(9,:), ':')
hold off
ylim([0.098, 0.105])

legend('LCP','BLCP','CCP','Convex')
xlabel('Time (sec)')
ylabel('Finger Gap (m)')

for i = 1:numel(hs)
    for j = 1:numel(hs(i).Children)
        hs(i).Children(j).LineWidth = 2;
    end
    hs(i).FontSize = 16;
    hs(i).FontWeight = 'bold';
    hs(i).Position(1) = 0.18;
    hs(i).Position(4) = 0.38;
end
f = gcf;
f.Position = [236   389 588 551];