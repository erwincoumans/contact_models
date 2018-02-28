%% Setup
clear

h = 0.01;
mu = 0.3;
m = 0.2;
m_g = 2.0;
r = 0.05;
w = 0.051;
lx = 0.01;
ly = 0.01;
lz = 0.02;
params = struct('h', h, 'mu', mu, 'm', m, 'm_g', m_g, 'r', r, 'w', w, ...
    'lx', lx, 'ly', ly, 'lz', lz, 'step_fun', []);

x0 = {[0, 0, r+0.15, 1, 0, 0, 0, zeros(1, 6)]'
      [0, 0, 0, 0, 0, 0]'
      [0, 0, 0, 0.2, 0, 0]'
      [0, 0, -0.001, 0, 0, 0]'
      [0, 0, -0.001, 1, 0, 0, 0, zeros(1, 6)]'
      [0, 0, r, 1, 0, 0, 0, 1.2*r, -1.2*r, 0, 0, 0, zeros(1,11)]'};
mus = {0.3; 0.3; 0.3; 0.3*ones(2,1); 0.9*ones(8,1); 0.3*ones(5,1)};
u = {@(x) zeros(6, 1)
     @(x) [-1, 0.6, 0]'
     @(x) [mu*9.81*m + 5*(0.2 - x(4)), 0, 0]'
     @(x) [1, 0, 0]'
     @(x) [2, 0, 0, 0, 0, 0]'
     @(x) [-4 4 0 0 5]'};
N = {6; 6; 6; 6; 21; 11};
steppers = {@step_sphere; @step_tooltip; @step_tooltip; ...
           @step_bead;   @step_box;     @step_gripper};
solvers = {@solver_lcp; @solver_blcp; @solver_ccp; @solver_convex};

%% Simulations
[dmean, dmax, dsd] = deal(NaN(numel(steppers), numel(solvers)));
for i = 1:numel(steppers)
    if (i == 5)
        params.w = 0.021;
    end
    params.mu = mus{i};
    for j = 1:numel(solvers)
        params.step_fun = solvers{j};
        [dm, x1, x2, f1, f2] = stepper_test(params, steppers{i}, x0{i}, u{i}, N{i});
        dmean(i,j) = mean(dm);
        dmax(i,j) = max(dm);
        dsd(i,j) = std(dm);
    end
end

%% Plotting
colors = [0,0.447,0.741;0.850,0.325,0.098;0.929,0.694,0.125;0.494,0.184,0.556];
b = bar(dmean);
for i = 1:numel(b)
    b(i).FaceColor = colors(i,:);
end
% hold on
% e = errorbar(dmean,dsd,'k.');
% e(1).XData = (1:6) - 0.27;
% e(2).XData = (1:6) - 0.09;
% e(3).XData = (1:6) + 0.09;
% e(4).XData = (1:6) + 0.27;

legend('LCP','BLCP','CCP','Convex')
ylabel('Discrepancy (m)')
a = gca;
a.YScale = 'log';
a.XTick = 1:6;
a.XTickLabel = {'Sphere','Sliding 1','Sliding 2','Sliding 3','Wedging','Grasp'};
a.XTickLabelRotation = 45;
a.FontSize = 16;
a.FontWeight = 'bold';
xlim([0,7])

f = gcf;
f.Position = [234 536 738 547];