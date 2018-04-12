% Spheres in a box.
clear

% Parameters
h = 0.01;
mu = 0.3;
m = 0.2;
r = 0.05;

N = 101;
time = 0:h:h*(N-1);

ctol = 0.1;

solvers = {@solver_ncp, @solver_blcp, @solver_ccp, @solver_convex};
zs = 1:15;

[res_t, res_p] = deal(NaN(numel(zs), numel(solvers)));

for kZ = 1:numel(zs)
    nx = 1;
    nz = zs(kZ);
    n = nx*nx*nz;

    rng('default')
    [x0, y0, z0] = meshgrid(3*(1:nx)-1.5, 3*(1:nx)-1.5, 3*(1:nz)-1.5);
    q0 = zeros(7*n, 1);
    q0(1:7:7*n) = r*x0(:) + r*(rand(n,1) - 0.5);
    q0(2:7:7*n) = r*y0(:) + r*(rand(n,1) - 0.5);
    q0(3:7:7*n) = r*z0(:) + r*(rand(n,1) - 0.5);
    q0(4:7:7*n) = 1;

    for kS = 1:numel(solvers)

        v = zeros(6*n, 1);
        Fext = repmat(9.81*m*[0; 0; -1; 0; 0; 0], n, 1);
        M = kron(eye(n), diag(m*[1 1 1 (2/5)*r^2 (2/5)*r^2 (2/5)*r^2]));
        I = m*(2/5)*r^2;

        q = q0;
    %     qs = cell(1,N);
    %     qs{1} = q;
        ttot = 0;
        for k = 2:N
            J = cell(1, n*(n-1)/2 + 3*n);
            psi = cell(1, n*(n-1)/2 + 3*n);
            nc = 1;
            for i = 1:n
                for j = (i+1):n
                    dij =    q(7*(j-1)+(1:3)) - q(7*(i-1)+(1:3));
                    vij = h*(v(6*(j-1)+(1:3)) - v(6*(i-1)+(1:3)));
                    t = max(0, min(1, dot(-dij, vij)/(h*norm(vij))));
                    if (norm(dij + t*vij) - 2*r < ctol)
                        psi{nc} = norm(dij) - 2*r;
                        nhat = dij/norm(dij);
                        if abs(nhat(1)) > abs(nhat(2))
                            that = cross(nhat, [0; 1; 0]);
                        else
                            that = cross(nhat, [1; 0; 0]);
                        end
                        that = that/norm(that);
                        ohat = cross(nhat, that);
                        G = zeros(3, 6*n);
                        rxf = [zeros(3,1) r*cross(that, nhat) r*cross(ohat, nhat)]';
                        G(:, 6*(j-1)+(1:3)) = +[nhat that ohat]';
                        G(:, 6*(i-1)+(1:3)) = -[nhat that ohat]';
                        G(:, 6*(j-1)+(4:6)) = rxf*quat2rotm(q(7*(j-1)+(4:7))');
                        G(:, 6*(i-1)+(4:6)) = rxf*quat2rotm(q(7*(i-1)+(4:7))');
                        J{nc} = G;
                        nc = nc + 1;
                    end
                end
            end
            for i = 1:n
                x = q(7*(i-1)+(1:3));
                dx = q(6*(i-1)+(1:3));
                R = quat2rotm(q(7*(i-1)+(4:7))');
                t = max(0, min(1, (r - x(1))/dx(1)));
                if (abs(x(1) + t*dx(1) - r) < ctol)
                    psi{nc} = x(1) - r;
                    G = zeros(3, 6*n);
                    G(:, 6*(i-1)+(1:3)) = [ 1  0  0;  0  1  0;  0  0  1];
                    G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0  0 -r;  0  r  0]*R;
                    J{nc} = G;
                    nc = nc + 1;
                end
                t = max(0, min(1, (r - x(2))/dx(2)));
                if (abs(x(2) + t*dx(2) - r) < ctol)
                    psi{nc} = x(2) - r;
                    G = zeros(3, 6*n);
                    G(:, 6*(i-1)+(1:3)) = [ 0  1  0; -1  0  0;  0  0  1];
                    G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0  0 -r; -r  0  0]*R;
                    J{nc} = G;
                    nc = nc + 1;
                end
                t = max(0, min(1, (r - x(3))/dx(3)));
                if (abs(x(3) + t*dx(3) - r) < ctol)
                    psi{nc} = x(3) - r;
                    G = zeros(3, 6*n);
                    G(:, 6*(i-1)+(1:3)) = [ 0  0  1;  1  0  0;  0  1  0];
                    G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0 -r  0;  r  0  0]*R;
                    J{nc} = G;
                    nc = nc + 1;
                end
                t = max(0, min(1, ((3*nx*r - r) - x(1))/dx(1)));
                if (abs(x(1) + t*dx(1) - (3*nx*r - r)) < ctol)
                    psi{nc} = 3*nx*r - r - x(1);
                    G = zeros(3, 6*n);
                    G(:, 6*(i-1)+(1:3)) = [-1  0  0;  0 -1  0;  0  0  1];
                    G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0  0 -r;  0 -r  0]*R;
                    J{nc} = G;
                    nc = nc + 1;
                end
                t = max(0, min(1, (3*nx*r - r - x(2))/dx(2)));
                if (abs(x(2) + t*dx(2) - (3*nx*r - r)) < ctol)
                    psi{nc} = 3*nx*r - r - x(2);
                    G = zeros(3, 6*n);
                    G(:, 6*(i-1)+(1:3)) = [ 0 -1  0;  1  0  0;  0  0  1];
                    G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0  0 -r;  r  0  0]*R;
                    J{nc} = G;
                    nc = nc + 1;
                end
            end

            psi = cat(1, psi{:});
            J = cat(1, J{:});
            J = cat(1, J(1:3:end,:), J(2:3:end,:), J(3:3:end,:));

            % Gravitational, external, and other forces
            for i = 1:n
                omega = v(6*(i-1)+(4:6));
                Fext(6*(i-1)+(4:6)) = -cross(omega, I*omega);
            end

            tic
            [v, imp] = solvers{kS}(v, Fext, M, J, repmat(mu, nc-1, 1), psi, h);
            tsolve = toc;
            ttot = ttot + tsolve;

            for i = 1:n
                q(7*(i-1)+(1:7)) = int_body(q(7*(i-1)+(1:7)), v(6*(i-1)+(1:6)), h);
            end
    %         qs{k} = q;
        end
    %     qw = [qs{:}];
    res_t(kZ, kS) = ttot;
    res_p(kZ, kS) = min(psi);
    fprintf('%s\t%d\t%g\t%g\n', func2str(solvers{kS}), n, ttot, min(psi))
    end
end

%% Error
h_p = semilogy(zs,-min(res_p,0));
h_p(1).LineStyle = '-';
h_p(2).LineStyle = '-.';
h_p(3).LineStyle = '--';
h_p(4).LineStyle = ':';

legend('NCP','BLCP','CCP','Convex')
xlabel('Number of Spheres')
ylabel('Penetration Error (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';

%% Timing
h_p = plot(zs,res_t);
h_p(1).LineStyle = '-';
h_p(2).LineStyle = '-.';
h_p(3).LineStyle = '--';
h_p(4).LineStyle = ':';

legend('NCP','BLCP','CCP','Convex')
xlabel('Number of Spheres')
ylabel('Computation Time (sec)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';