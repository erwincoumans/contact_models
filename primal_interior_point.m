function f = primal_interior_point(H, c, A, b, nl, nc)
% Attempts to solve:
%   f = argmin 0.5*f'*H*f + c'*f  subject to  A*f >= b

H = 0.5*(H + H');

% Parameters
tol = 1e-8;
kappa = 0.00001;
q = 10;
s0 = kappa/q;

% Modified constraints (finite everywhere)
a0 = 0.5*kappa/s0^2;
a1 = -kappa/s0 - 2*a0*s0;
a2 = -kappa*log(s0) - a0*s0^2 - a1*s0;

% Initial guess for impulses
f = ones(size(c));

% Satify friction cone and positive impulse constraints
f(1:nl+nc) = max(f(1:nl+nc), 2*s0);
f(nl+nc+1:nl+2*nc,1) = sign(f(nl+nc+(1:nc),1)) .* min(abs(f(nl+nc+(1:nc),1)),...
    0.5*A(nl+nc+(1:nc),nl+(1:nc)) * f(nl+(1:nc),1));

iter = 0;
while (true)
    % Modify constraints if necessary
    s = A*f - b;
    g_vals = -kappa*s.^-1;
    g_vals(s <= tol) = 2*a0*s(s <= tol) + a1;
    h_vals = kappa*s.^-2;
    h_vals(s <= tol) = 2*a0;
    
    % Derivatives
    grad = H*f + c + A'*g_vals;
    hess = H + A'*diag(h_vals)*A;

    % Newton's method
    df = -hess\grad;
    
    % Stopping criteria
    if (norm(df) < tol)
        break
    end

    % Line search (exact)
    scale = 1;
    dconstraint = (A*df)./(A*f - b);
    if any(dconstraint < -1)
        scale = -1/min(dconstraint);
    end

    f = f + scale*df;
    iter = iter + 1;
end
fprintf('Iterations: %d\n', iter);

end