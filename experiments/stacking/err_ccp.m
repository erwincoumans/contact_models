function err = err_ccp(v_prev, Fext, M, J, mu, psi, h, niter)
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

%% Cone Complementarity Problem (CCP)
d = diag(A);
d = mean(reshape(d, nc, 3), 2);

nto = [0, nc, 2*nc]; % normal and tangent indices

% Solve for contact impulses (Block Projected Gauss-Seidel)
x = zeros(3*nc,1);
xs = cell(1,niter);
for r = 1:niter
    xs{r} = x;
    for i = 1:nc
        % Block update
        xstar = x(i+nto) - (A(i+nto,:)*x + btilde(i+nto))/d(i);
        x(i+nto) = project_cone(xstar, mu(i));
    end
end

%% Calculate error
xs = [xs{:}];
% err = sqrt(sum(bsxfun(@minus, xs, xs(:,end)).^2))/norm(xs(:,end));
err = min(A(1:nc,:)*xs + btilde(1:nc));

end

function xproj = project_cone(x, mu)
% Project impulse into friction cone

x_n = x(1); % normal
x_f = norm(x(2:end)); % combined friction

if x_f <= mu*x_n
    % x is already in the cone
    xproj = x;
elseif x_f <= -x_n/mu
    % x is in the polar cone
    xproj = zeros(size(x));
else
    pi_n = (x_f*mu + x_n)/(mu^2 + 1);
    xproj = [pi_n; mu*x(2:end)*pi_n/x_f];
end
end