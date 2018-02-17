function x0 = interior_point(H, c, A, b, mu)
% Attempts to solve:
%   x = argmin 0.5*x'*H*x + x'*c  subject to  A*x >= b

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
        % Modify constraints if necessary
        s = [A*x0 - b; mu.^2.*x0(1:nc).^2 - x0(nc+1:2*nc).^2 - x0(2*nc+1:3*nc).^2];
        G = [A; diag(2*mu.^2.*x0(1:nc)) diag(-2*x0(nc+1:2*nc)) diag(-2*x0(2*nc+1:3*nc))];
        dG = [zeros(size(A)); diag(2*mu.^2) -2*eye(nc) -2*eye(nc)];
        mask = s < s0;
        g_vals = -kappa*s.^-1;
        g_vals(mask) = 2*a0*s(mask) + a1;
        h_vals = kappa*s.^-2;
        h_vals(mask) = 2*a0;

        % Derivatives
        grad = H*x0 + c + G'*g_vals;
        hess = H + G'*diag(h_vals)*G + diag(g_vals'*dG);

        % Newton's method
        if rcond(hess) < 1e-8
            dx = -pinv(hess)*grad;
        else
            dx = -hess\grad;
        end

        % Line search (exact)
        dc = (s0 - A(~mask(1:2*nc),:)*x0 + b(~mask(1:2*nc)))./(A(~mask(1:2*nc),:)*dx) - eps;
        dc = dc(dc >= 0);
        t = min([dc;1]);
        % For impulses in the friction cone
        for i = find(~mask(nc+1:2*nc) & ~mask(2*nc+1:3*nc))
            p = x(i+[0,nc,2*nc]);
            dp = dx(i+[0,nc,2*nc]);
            % If search direction eventually leavs the friction cone
            if (norm(dp(2:3)) > mu(i)*dp(1))
                % Find intersection
                q = [mu(i)^2; -1; -1];
                c0 = dot(q.*dp, dp);
                c1 = 2*dot(q.*dp, p);
                c2 = dot(q.*p, p) - s0;
                % Take "+" root
                t = min(t, (-c1 + sqrt(c1^2 - 4*c0*c2))/(2*c0));
            end
        end

        % Line search (backtracking)
        v = sum(log(s(~mask))) - sum(a0*s(mask).^2 + a1*s(mask) + a2);
        f0 = 0.5*x0'*H*x0 + x0'*c - kappa*v;
        while (true)
            x = x0 + t*dx;
            s = [A*x - b; mu.^2.*x(1:nc).^2 - x(nc+1:2*nc).^2 - x(2*nc+1:3*nc).^2];
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