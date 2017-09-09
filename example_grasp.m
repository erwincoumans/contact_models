% Grasp using constant control inputs.

% Note: with convex contact model, the gripper opens a bit when it hits
% the high stop. Using h=0.5 exacerbates the issue.

r = 0.5;
x0 = [0, r, 0, 2*r, -2*r, 0, zeros(1,6)]';
u = [-4 4 2]';
N = 100;
[x, f] = gripper_sim(x0, repmat(u,1,100));

lstyle = '-';
plot(x(2,:), ['b' lstyle])
hold on
plot(x(4,:) - x(5,:), ['g' lstyle])
plot(x(6,:), ['r' lstyle])
hold off