% Stack spheres in a box to characterize solver error.
clear

% Parameters
h = 0.01;
mu = 0.3;
m = 0.2;
r = 0.05;
N = 101;

%% (1) Simulate one trajectory
time = 0:h:h*(N-1);

nx = 1;
nz = 4;
n = nx*nx*nz;

rng('default')
M = kron(eye(n), diag(m*[1 1 1 (2/5)*r^2 (2/5)*r^2 (2/5)*r^2]));
q = init_sphere_configs(r, nx, nz, n);

v = zeros(6*n, 1);
qs = cell(1, N);
qs{1} = q;
for k = 2:N
    [psi, J] = get_sphere_jacobians(q, v, h, r, nx, n);
    Fext = get_sphere_forces(v, m, r, n);

    [v, ~] = solver_blcp(v, Fext, M, J, repmat(mu, size(psi)), psi, h);

    for i = 1:n
        q(7*(i-1)+(1:7)) = int_body(q(7*(i-1)+(1:7)), v(6*(i-1)+(1:6)), h);
    end
    qs{k} = q;
end
qs = [qs{:}];

% Animation
plot_spheres(qs, r, nx, nz);

%% (2) Simulate trajectories with different models and stack heights
solvers = {@solver_ncp, @solver_blcp, @solver_ccp, @solver_convex};
nmax = 15;

[tsolve, perr] = deal(NaN(nmax, numel(solvers)));

rng('default')
for kN = 1:nmax
    nx = 1;
    nz = kN;
    n = nx*nx*nz;
    
    M = kron(eye(n), diag(m*[1 1 1 (2/5)*r^2 (2/5)*r^2 (2/5)*r^2]));
    q0 = init_sphere_configs(r, nx, nz, n);

    for kS = 1:numel(solvers)
        ttot = 0;
        v = zeros(6*n, 1);
        q = q0;
        for k = 2:N
            [psi, J] = get_sphere_jacobians(q, v, h, r, nx, n);
            Fext = get_sphere_forces(v, m, r, n);
            
            tic
            [v, ~] = solvers{kS}(v, Fext, M, J, repmat(mu, size(psi)), psi, h);
            ttot = ttot + toc;

            for i = 1:n
                q(7*(i-1)+(1:7)) = int_body(q(7*(i-1)+(1:7)), v(6*(i-1)+(1:6)), h);
            end
        end
        tsolve(kN, kS) = ttot;
        perr(kN, kS) = min(psi);
        fprintf('%s\t%d\t%g\t%g\n', func2str(solvers{kS}), n, ttot, min(psi))
    end
end

% Plot penetration error
figure(1)
h_p = semilogy(1:nmax,-min(perr,0));
h_p(1).LineStyle = '-';
h_p(2).LineStyle = '-.';
h_p(3).LineStyle = '--';
h_p(4).LineStyle = ':';

legend({'NCP','BLCP','CCP','Convex'},'Location','Southeast')
xlabel('Number of Spheres')
ylabel('Penetration Error (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';

% Plot computation times
figure(2)
h_p = plot(1:nmax,tsolve);
h_p(1).LineStyle = '-';
h_p(2).LineStyle = '-.';
h_p(3).LineStyle = '--';
h_p(4).LineStyle = ':';

legend({'NCP','BLCP','CCP','Convex'},'Location','Northwest')
xlabel('Number of Spheres')
ylabel('Computation Time (sec)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';

%% (3) Simulate trajectories with different models and stack heights
solvers = {@solver_ncp, @solver_blcp, @solver_ccp, @solver_convex};
% MODIFICATION
%{
function err = solver_...
...
xs = cell(1,1000);
for r = 1:1000
    xs{r} = x;
    ...
end
xs = [xs{:}];
err = sqrt(sum(bsxfun(@minus, xs, xs(:,end)).^2))/norm(xs(:,end));
err = min(A(1:nc,:)*xs + btilde(1:nc));
%}

err = cell(1, numel(solvers));

rng('default')
nx = 1;
nz = 4;
n = nx*nx*nz;

M = kron(eye(n), diag(m*[1 1 1 (2/5)*r^2 (2/5)*r^2 (2/5)*r^2]));
q0 = init_sphere_configs(r, nx, nz, n);

v = zeros(6*n, 1);
q = q0;
for k = 2:N
    [psi, J] = get_sphere_jacobians(q, v, h, r, nx, n);
    Fext = get_sphere_forces(v, m, r, n);

    [v, ~] = solver_lcp(v, Fext, M, J, repmat(mu, size(psi)), psi, h);

    for i = 1:n
        q(7*(i-1)+(1:7)) = int_body(q(7*(i-1)+(1:7)), v(6*(i-1)+(1:6)), h);
    end
end

for kS = 1:numel(solvers)
    [psi, J] = get_sphere_jacobians(q, v, h, r, nx, n);
    Fext = get_sphere_forces(v, m, r, n);

    err{kS} = solvers{kS}(v, Fext, M, J, repmat(mu, size(psi)), psi, h);
end

%% Plot error
styles = {'-','-.','--',':'};
clf
hold on
for kS = 1:numel(solvers)
    plot(1:size(err{kS},2), err{kS}, styles{kS})
end
legend({'NCP','BLCP','CCP','Convex'},'Location','Southwest')

xlabel('Iteration')
ylabel('Penetration Error (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.YScale = 'log';
a.YDir = 'reverse';
set(gcf,'Position',[676   838   333   256])
ylim([-1e-0 -1e-18])