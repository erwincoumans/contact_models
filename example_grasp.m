% Grasp using constant control inputs.

% Note: with convex contact model, the gripper opens a bit when it hits
% the high stop. Larger h exacerbates the issue.
clear

% Parameters
h = 0.01;
mu = 0.3*ones(4,1);
m = 0.2;
r = 0.05;
m_g = 2.0;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_g', m_g, 'step_fun', []);

x0 = [0, 0, r, 1, 0, 0, 0, 1.2*r, -1.2*r, 0, 0, 0, zeros(1,11)]';
u = [-14 14 0 0 10]';
N = 51;

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

%% Sphere Height
figure(1)
plot(time, x1(3,:), '-')
hold on
plot(time, x2(3,:), '-.')
plot(time, x3(3,:), '--')
plot(time, x4(3,:), ':')
hold off

legend('LCP','BLCP','CCP','Convex')
xlabel('Time (sec)')
ylabel('Sphere Height (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';

%% Slip Velocity
figure(2)
idx = find(time>=0.2);
plot(time(idx:end),x1(15,idx:end) - x1(23,idx:end), '-')
hold on
plot(time(idx:end),x2(15,idx:end) - x2(23,idx:end), '-.')
plot(time(idx:end),x3(15,idx:end) - x3(23,idx:end), '--')
hold off

legend('LCP','BLCP','CCP')
xlabel('Time (sec)')
ylabel('Slip Velocity (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';

%% Finger Separation
figure(3)
plot(time, x1(8,:)-x1(9,:), '-')
hold on
plot(time, x2(8,:)-x2(9,:), '-.')
plot(time, x3(8,:)-x3(9,:), '--')
plot(time, x4(8,:)-x4(9,:), ':')
hold off

legend('LCP','BLCP','CCP','Convex')
xlabel('Time (sec)')
ylabel('Finger Separation (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';