function err = err_ncp(v_prev, Fext, M, J, mu, psi, h, niter)
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

%% Nonlinear Complementarity Problem (NCP)
d = diag(A);
d(nc+1:2*nc) = min(d(nc+1:2*nc), d(2*nc+1:3*nc));

to = [nc, 2*nc]; % tangent indices

% Solve for contact impulses (Proximal Point)
x = zeros(3*nc,1);
xs = cell(1,niter);
for r = 1:niter
    xs{r} = x;
    for i = 1:nc
        % Normal
        xstar = x(i) - (A(i,:)*x + btilde(i))/d(i);
        x(i) = max(0, xstar);

        % Friction
        xstar = x(i+to) - (A(i+to,:)*x + btilde(i+to))/d(i+nc);
        x(i+to) = project_circle(xstar, mu(i)*x(i));
    end
end

%% Calculate error
xs = [xs{:}];
% err = sqrt(sum(bsxfun(@minus, xs, xs(:,end)).^2))/norm(xs(:,end));
err = min(A(1:nc,:)*xs + btilde(1:nc));

end

function xproj = project_circle(x, r)
% Project frictional impulse into a circle of radius r

x_f = norm(x); % combined friction

if x_f <= r
    xproj = x;
else
    xproj = x*(r/x_f);
end
end