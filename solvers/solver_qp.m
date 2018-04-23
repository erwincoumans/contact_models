function [v_next, x] = solver_qp(v_prev, Fext, M, J, mu, psi, h)
% Input:
%   v_prev - velocity [n x 1]
%   Fext - gravitational and other forces [n x 1]
%   M - inertia matrix [n x n]
%   J - contact Jacobian [3*nc x n]
%    (all normal, 1st tangent, and 2nd tangent directions in that order)
%   mu - coefficients of friction [nc x 1]
%   psi - contact gap distances [nc x 1]
%   h - time step size
% Output:
%   v_next - velocity [n x 1]
%   x - contact impulses [3*nc x 1]

%% Setup
nc = size(mu,1); % number of contacts

% Inverse inertia matrix in the contact frame
A = J*(M\J');
A = (A + A')/2; % should be symmetric

% Resulting contact velocities if all contact impulses are 0
b = J*(v_prev + M\Fext*h);

% Baumgarte stabilization
btilde = b + [psi/h; zeros(2*nc,1)];

%% Convex Quadratic Program

% Contact smoothing
Rmax = 100;
Rmin = 0.01;
wmax = 0.1;
R = diag(Rmin + (Rmax - Rmin)*[psi; psi; psi]/wmax);

% Constraints
Ac = [A(1:nc,:);... % no penetration
      eye(nc)  zeros(nc,2*nc)]; % no attractive contact forces
bc = [-btilde(1:nc); zeros(nc,1)];

% Solve for contact impulses (Interior-point)
x = interior_point(A + R, b, Ac, bc, mu);

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

end

function x0 = interior_point(H, c, A, b, mu)
% Attempts to solve:
%   x = argmin 0.5*x'*H*x + x'*c  subject to  A*x >= b and friction cones
%
% Note: a particular problem structure is assumed of H, c, x, ...

nc = size(mu,1); % number of contacts

H = 0.5*(H + H');

% Parameters
tol1 = 1e-10;
tol2 = 1e-4;
kappa = 0.1;
alpha = 0.01;
beta = 0.5;
eta = 10;
q = 10;

% Initial guess for impulses
x0 = zeros(size(c));
x0(1:nc) = kappa/q;

iter = 0;
while (3*nc*kappa > tol1)
    % Modified constraints (finite everywhere)
    s0 = kappa/q;
    a0 = 0.5*kappa/s0^2;
    a1 = -kappa/s0 - 2*a0*s0;
    a2 = -kappa*log(s0) - a0*s0^2 - a1*s0;

    x_prev = x0 + 3*tol2;
    while (max(abs(x0 - x_prev)) > tol2)
        x_prev = x0;

        % Constraints
        s = [A*x0 - b; mu.^2.*x0(1:nc).^2 - x0(nc+1:2*nc).^2 - x0(2*nc+1:3*nc).^2];

        % Modify violated constraints
        mask = s < 0;
        g_vals = -kappa*s.^-1;
        g_vals(mask) = 2*a0*s(mask) + a1;
        h_vals = kappa*s.^-2;
        h_vals(mask) = 2*a0;

        % Derivatives
        G = [A; diag(2*mu.^2.*x0(1:nc)) diag(-2*x0(nc+1:2*nc)) diag(-2*x0(2*nc+1:3*nc))];
        dG = [zeros(size(A)); diag(2*mu.^2) -2*eye(nc) -2*eye(nc)];
        grad = H*x0 + c + G'*g_vals;
        hess = H + G'*diag(h_vals)*G + diag(g_vals'*dG);

        % Newton's method
        if rcond(hess) < 1e-8
            dx = -pinv(hess)*grad;
        else
            dx = -hess\grad;
        end

        % Line search (backtracking)
        v = sum(log(s(~mask))) - sum(a0*s(mask).^2 + a1*s(mask) + a2);
        f0 = 0.5*x0'*H*x0 + x0'*c - kappa*v;
        t = 1;
        while (true)
            x = x0 + t*dx;
            s = [A*x - b; mu.^2.*x(1:nc).^2 - x(nc+1:2*nc).^2 - x(2*nc+1:3*nc).^2];
            % Check that no new constraint violations are introduced
            if ~any(s < 0 & ~mask)
                v = sum(log(s(~mask))) - sum(a0*s(mask).^2 + a1*s(mask) + a2);
                f = 0.5*x'*H*x + x'*c - kappa*v;
                if (f <= f0 + alpha*t*(grad'*dx))
                    break
                end
            end
            t = beta*t;
        end

        x0 = x0 + t*dx;
        iter = iter + 1;
    end
    kappa = kappa/eta;
end

s = [A*x0 - b; mu.^2.*x0(1:nc).^2 - x0(nc+1:2*nc).^2 - x0(2*nc+1:3*nc).^2];
if any(s < 0)
    warning('Interior-point method terminated with unsatisfied constraints.')
end
% fprintf('Iterations: %d\n', iter);

end