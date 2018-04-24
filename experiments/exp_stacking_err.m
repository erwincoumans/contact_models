% Stack spheres in a box to characterize solver error.
clear

% Parameters
h = 0.01;
mu = 0.3;
m = 0.2;
r = 0.05;
N = 101;

%% Simulate spheres stacking
rng('default')
nx = 1; % stack width
nz = 4; % stack height
n = nx*nx*nz;

M = kron(eye(n), diag(m*[1 1 1 (2/5)*r^2 (2/5)*r^2 (2/5)*r^2]));
q0 = init_sphere_configs(r, nx, nz, n);

v = zeros(6*n, 1);
q = q0;
for k = 2:N
    [psi, J] = get_sphere_jacobians(q, v, h, r, nx, n);
    Fext = get_sphere_forces(v, m, r, n);
    
    % Use direct solver so penetration error is near machine precision
    [v, ~] = solver_lcp(v, Fext, M, J, repmat(mu, size(psi)), psi, h);

    for i = 1:n
        q(7*(i-1)+(1:7)) = int_body(q(7*(i-1)+(1:7)), v(6*(i-1)+(1:6)), h);
    end
end

%% Simulate one step with different iterative solvers
solvers = {@err_ncp, @err_blcp, @err_ccp, @err_convex};
niter = 1000;

[psi, J] = get_sphere_jacobians(q, v, h, r, nx, n);
Fext = get_sphere_forces(v, m, r, n);
fprintf('solver_lcp\t%d\t?\t%e\n', n, min(psi))

err = cell(1, numel(solvers));
for kS = 1:numel(solvers)
    err{kS} = solvers{kS}(v, Fext, M, J, repmat(mu, size(psi)), psi, h, niter);
end
err = cat(1, err{:});

%% Plot error
plot(1:niter, err(1,:), '-')
hold on
plot(1:niter, err(2,:), '-.')
plot(1:niter, err(3,:), '--')
plot(1:niter, err(4,:), ':')
hold off

legend({'NCP','BLCP','CCP','Convex'}, 'Location', 'Southwest')
xlabel('Iteration')
ylabel('Penetration Error (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.YScale = 'log';
a.YDir = 'reverse';
ylim([-1e-0 -1e-18])