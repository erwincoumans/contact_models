% Grasp using constant control inputs.

% Note: with convex contact model, the gripper opens a bit when it hits
% the high stop. Larger h exacerbates the issue.
clear

% Parameters
h = 0.02;
mu = [0.3; 0.3; 0.2];
m = 0.2;
r = 0.05;
m_g = 2.0;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_g', m_g, 'step_fun', @forward_ccp);

x0 = [0, r, 0, 1.2*r, -1.2*r, 0, zeros(1,6)]';
u = [-4 4 5]';
N = 50;

%% Simulation
time = 0:h:h*N;

params.step_fun = @forward_lcp;
[x{1}, ~] = gripper_sim(params, x0, repmat(u,1,N));
params.step_fun = @forward_ccp;
[x{2}, ~] = gripper_sim(params, x0, repmat(u,1,N));
params.step_fun = @forward_convex;
[x{3}, ~] = gripper_sim(params, x0, repmat(u,1,N));

%% Plotting
colors = {[0 0.447 0.741], [0.850 0.325 0.098], [0.929 0.6940 0.125]};
styles = {'-', '--', ':'};

for i = 1:3
    plot(time, x{i}(2,:), styles{i}, 'Color', colors{1})
    hold on
    plot(time, x{i}(4,:) - x{i}(5,:), styles{i}, 'Color', colors{2})
    plot(time, x{i}(6,:), styles{i}, 'Color', colors{3})
end
hold off

legend('Disk Y-Position', 'Gripper Gap', 'Gripper Y-Position')
xlabel('Time (sec)')
ylabel('Position (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';