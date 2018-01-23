% Grasp using constant control inputs.
clear

% Parameters
h = 0.02;
mu = 0.2*ones(4,1);
m = 0.2;
r = 0.02;
l = 0.05;
w = 0.021;

params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'l', l, ...
    'w', w, 'step_fun', @forward_lcp);

x0 = [0, 0, 0, 0, 0, 0]';
u = [0, 0, 0]';
N = 51;

%% Simulation
time = 0:h:h*(N-1);
[x1, x2, x3] = deal(repmat(x0, 1, N));
[f1, f2, f3] = deal(cell(1, N));

% params.step_fun = @forward_lcp;
% for k = 2:N
%     [x1(:,k), f1{k}] = peg_step(params, x1(:,k-1), [2 0 0]');
% end
% f1 = [f1{:}];

params.step_fun = @forward_ccp;
for k = 2:N
    [x2(:,k), f2{k}] = peg_step(params, x2(:,k-1), [2 0 0]');
end
f2 = [f2{:}];

params.step_fun = @forward_convex;
for k = 2:N
    [x3(:,k), f3{k}] = peg_step(params, x3(:,k-1), [2 0 0]');
end
f3 = [f3{:}];

% %% Plotting
% plot(time, x1(2,:), '-')
% hold on
% plot(time, x2(2,:), '--')
% plot(time, x3(2,:), ':')
% hold off
% 
% legend('LCP','CCP','Convex')
% xlabel('Time (sec)')
% ylabel('Particle Y-Position (m)')
% a = gca;
% for k = 1:numel(a.Children)
%     a.Children(k).LineWidth = 2;
% end
% a.FontSize = 14;
% a.FontWeight = 'bold';