% Grasp using constant control inputs.

% Note: with convex contact model, the gripper opens a bit when it hits
% the high stop. Larger h exacerbates the issue.
clear

% Parameters
h = 0.02;
mu = [0.2; 0.2; 0.3; 0.3; 0.2];
m = 0.2;
r = 0.05;
m_g = 2.0;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_g', m_g, 'step_fun', []);

x0 = [0, 0, r, 1, 0, 0, 0, 1.2*r, -1.2*r, 0, 0, 0, zeros(1,11)]';
u = [-4 4 0 0 5]';
N = 51;

%% Simulation
time = 0:h:h*(N-1);

params.step_fun = @solver_lcp;
[x{1}, f{1}] = stepper(params, @step_gripper, x0, u, N);
params.step_fun = @solver_blcp;
[x{2}, f{2}] = stepper(params, @step_gripper, x0, u, N);
params.step_fun = @solver_ccp;
[x{3}, f{3}] = stepper(params, @step_gripper, x0, u, N);
params.step_fun = @solver_convex;
[x{4}, f{4}] = stepper(params, @step_gripper, x0, u, N);

%% Plotting
colors = {[0 0.447 0.741], [0.850 0.325 0.098], [0.929 0.6940 0.125], [0.494 0.184 0.556]};
styles = {'-', '-.', '--', ':'};

for i = 1:4
    plot(time, x{i}(3,:), styles{i}, 'Color', colors{1})
    hold on
    plot(time, x{i}(8,:) - x{i}(9,:), styles{i}, 'Color', colors{2})
    plot(time, x{i}(12,:), styles{i}, 'Color', colors{3})
end
hold off

legend('Sphere Height', 'Gripper Gap', 'Gripper Height')
xlabel('Time (sec)')
ylabel('Position (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';