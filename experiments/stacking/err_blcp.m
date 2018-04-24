function err = err_blcp(v_prev, Fext, M, J, mu, psi, h, niter)
% Input:
%   v_prev - velocity [n x 1]
%   Fext - gravitational and other forces [n x 1]
%   M - inertia matrix [n x n]
%   J - contact Jacobian [3*nc x n]
%    (all normal, 1st tangent, and 2nd tangent directions in that order)
%   mu - coefficients of friction [nc x 1]
%   psi - contact gap distances [nc x 1]
%   h - time step size
%   niter - number of iterations
% Output:
%   err - solver error [1 x niter]

%% Setup
nc = size(mu,1); % number of contacts

% Inverse inertia matrix in the contact frame
A = J*(M\J');
A = (A + A')/2; % should be symmetric

% Resulting contact velocities if all contact impulses are 0
b = J*(v_prev + M\Fext*h);

% Baumgarte stabilization
btilde = b + [psi/h; zeros(2*nc,1)];

%% Bounded Linear Complementarity Problem (BLCP)
d = diag(A);

% Solve for contact impulses (Projected Gauss-Seidel)
x = zeros(3*nc,1);
xs = cell(1,niter);
for r = 1:niter
    xs{r} = x;
    for i = 1:nc
        % Normal (n)
        xstar = x(i) - (A(i,:)*x + btilde(i))/d(i);
        x(i) = max(0, xstar);
        lim = mu(i)*x(i);

        % Tangent (t)
        xstar = x(i+1*nc) - (A(i+1*nc,:)*x + btilde(i+1*nc))/d(i+1*nc);
        x(i+1*nc) = min(max(-lim, xstar), lim);

        % Tangent (o)
        xstar = x(i+2*nc) - (A(i+2*nc,:)*x + btilde(i+2*nc))/d(i+2*nc);
        x(i+2*nc) = min(max(-lim, xstar), lim);
    end
end

%% Calculate error
xs = [xs{:}];
% err = sqrt(sum(bsxfun(@minus, xs, xs(:,end)).^2))/norm(xs(:,end));
err = min(A(1:nc,:)*xs + btilde(1:nc));

end