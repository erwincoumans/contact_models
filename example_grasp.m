% Grasp using constant control inputs.

% Note: with convex contact model, the gripper opens a bit when it hits
% the high stop. Using h=0.5 exacerbates the issue.

% Parameters
h = 0.02;
mu = [0.3; 0.3; 0.2];
m = 0.1;
r = 0.5;
m_g = 0.8;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_g', m_g, 'step_fun', @forward_convex);

x0 = [0, r, 0, 2*r, -2*r, 0, zeros(1,6)]';
u = [-4 4 2]';
N = 100;
[x, f] = gripper_sim(params, x0, repmat(u,1,100));

%% Lines
time = 0:h:h*N;
lstyle = '-';
plot(time, x(2,:), ['b' lstyle])
hold on
plot(time, x(4,:) - x(5,:), ['g' lstyle])
plot(time, x(6,:), ['r' lstyle])
hold off
legend('Disk Y-Position', 'Gripper Gap', 'Gripper Y-Position')
xlabel('Time (Seconds)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';