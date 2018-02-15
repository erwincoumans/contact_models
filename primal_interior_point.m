function x0 = primal_interior_point(H, c, A, b)
% Attempts to solve:
%   x = argmin 0.5*x'*H*x + x'*c  subject to  A*x >= b

H = 0.5*(H + H');

% Parameters
tol1 = 1e-10;
tol2 = 1e-4;
kappa = 0.1;
alpha = 0.01;
beta = 0.5;
eta = 10;
q = 10;

m = size(A,1);

% Initial guess for impulses
x0 = zeros(size(c));
iter = 0;

while (m*kappa > tol1)
    % Modified constraints (finite everywhere)
    s0 = kappa/q;
    a0 = 0.5*kappa/s0^2;
    a1 = -kappa/s0 - 2*a0*s0;
    a2 = -kappa*log(s0) - a0*s0^2 - a1*s0;
    
    x_prev = x0 + 3*tol2;
    while (max(abs(x0 - x_prev)) > tol2)
        x_prev = x0;
        % Modify constraints if necessary
        s = A*x0 - b;
        mask = s < s0;
        g_vals = -kappa*s.^-1;
        g_vals(mask) = 2*a0*s(mask) + a1;
        h_vals = kappa*s.^-2;
        h_vals(mask) = 2*a0;

        % Derivatives
        grad = H*x0 + c + A'*g_vals;
        hess = H + A'*diag(h_vals)*A;

        % Newton's method
        dx = -hess\grad;

        % Line search (exact)
        dc = (s0 - A(~mask,:)*x0 + b(~mask))./(A(~mask,:)*dx)-eps;
        dc = dc(dc >= 0);
        t = min([dc;1]);

        % Line search (backtracking)
        v = sum(log(s(~mask))) - sum(a0*s(mask).^2 + a1*s(mask) + a2);
        f0 = 0.5*x0'*H*x0 + x0'*c - kappa*v;
        while (true)
            x = x0 + t*dx;
            s = A*x - b;
            v = sum(log(s(~mask))) - sum(a0*s(mask).^2 + a1*s(mask) + a2);
            f = 0.5*x'*H*x + x'*c - kappa*v;
            if f <= f0 + alpha*t*(grad'*dx)
                break
            end
            t = beta*t;
        end

        x0 = x0 + t*dx;
        iter = iter + 1;
    end
    kappa = kappa/eta;
end

% fprintf('Iterations: %d\n', iter);

end