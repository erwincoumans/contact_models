% Stack spheres in a box to test scaling behaviors.
clear

% Parameters
h = 0.01;
mu = 0.3;
m = 0.2;
r = 0.05;
N = 101;

%% Simulate trajectories with different models and stack heights
solvers = {@solver_ncp, @solver_blcp, @solver_ccp, @solver_convex};
nmax = 15;

[tsolve, perr] = deal(NaN(nmax, numel(solvers)));

rng('default')
for kN = 1:nmax
    nx = 1; % stack width
    nz = kN; % stack height
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
        fprintf('%s\t%d\t%.2f\t%e\n', func2str(solvers{kS}), n, ttot, min(psi))
    end
end

% Plot penetration error
figure('Position',[1150 100 333 256])
hp = semilogy(1:nmax, -min(perr,0));
hp(1).LineStyle = '-';
hp(2).LineStyle = '-.';
hp(3).LineStyle = '--';
hp(4).LineStyle = ':';

legend({'NCP','BLCP','CCP','Convex'}, 'Location', 'Southeast')
xlabel('Number of Spheres')
ylabel('Penetration Error (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end

% Plot computation times
figure('Position',[1500 100 333 256])
hp = plot(1:nmax, tsolve);
hp(1).LineStyle = '-';
hp(2).LineStyle = '-.';
hp(3).LineStyle = '--';
hp(4).LineStyle = ':';

legend({'NCP','BLCP','CCP','Convex'}, 'Location', 'Northwest')
xlabel('Number of Spheres')
ylabel('Computation Time (sec)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end

%% Simulate one trajectory
time = 0:h:h*(N-1);

nx = 1; % stack width
nz = 4; % stack height
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
    
    [v, ~] = solver_convex(v, Fext, M, J, repmat(mu, size(psi)), psi, h);
    
    for i = 1:n
        q(7*(i-1)+(1:7)) = int_body(q(7*(i-1)+(1:7)), v(6*(i-1)+(1:6)), h);
    end
    qs{k} = q;
end
qs = [qs{:}];

% Animation
f = figure();
params = struct('h', h, 'r', r, 'nx', nx, 'nz', 8/3);
plot_spheres(params, qs(:,21));
f.Position(1:2) = [1150 450];
