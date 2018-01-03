% Grasp using constant control inputs.

% Note: with convex contact model, the gripper opens a bit when it hits
% the high stop. Larger h exacerbates the issue.

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
[x, f] = gripper_sim(params, x0, repmat(u,1,N));

%% Lines
time = 0:h:h*N;
lstyle = '-';
plot(time, x(2,:), lstyle, 'Color', [0 0.447 0.741])
hold on
plot(time, x(4,:) - x(5,:), lstyle, 'Color', [0.850 0.325 0.098])
plot(time, x(6,:), lstyle, 'Color', [0.929 0.6940 0.125])
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