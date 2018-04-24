function q0 = init_sphere_configs(r, nx, nz, n)
% Initial sphere positions with random perturbations

[x0, y0, z0] = meshgrid(3*(1:nx)-1.5, 3*(1:nx)-1.5, 3*(1:nz)-1.5);
q0 = zeros(7*n, 1);
q0(1:7:7*n) = r*x0(:) + r*(rand(n,1) - 0.5);
q0(2:7:7*n) = r*y0(:) + r*(rand(n,1) - 0.5);
q0(3:7:7*n) = r*z0(:) + r*(rand(n,1) - 0.5);
q0(4:7:7*n) = 1;
end