function [v_next, x] = solver_gpgs(v_prev, Fext, M, J, mu, psi, h)
% Input:
%   v_prev - velocity [n x 1]
%   Fext - gravitational and other forces [n x 1]
%   M - inertia matrix [n x n]
%   J - contact Jacobian [3*nc x n]
%   mu - coefficients of friction [nc x 1]
%   psi - contact gap distances [nc x 1]
%   h - time step
% Output:
%   v_prev - velocity [n x 1]
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
% min 0.5*x'*(A+R)*x + x'*btilde s.t. x in F_mu

% kappa = 1e-4;
epsilon = 1e-5;
r_min = 1e-7;

% bp = J*v_prev;
% btilde = b - bp + h*(1 + epsilon)*(2*kappa^(-1)*bp + kappa^(-2)*[psi; zeros(2*nc,1)]);

nt = [0, nc, 2*nc]; % normal and tangent indices

D = zeros(nc,1);
for i = 1:nc
    D(i) = trace(A(i+nt, i+nt))/3;
end

R = max(D*epsilon, r_min);
D = D + R;
A = A + diag(repmat(R, 3, 1));

% Solve for contact impulses (Projected Gauss-Seidel)
x = zeros(3*nc,1);
for r = 1:30
    for i = 1:nc
        % Block update
        xnew = x(i+nt) - (A(i+nt,:)*x + btilde(i+nt))/D(i);
        % Project impulse into friction cone
        x(i+nt) = project(xnew, mu(i));
    end
end

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

end

function xproj = project(x, mu)
% Project impulse into friction cone

x_n = x(1); % normal
x_r = norm(x(2:end)); % combined radial

if x_r <= mu*x_n
    % x is already in the cone
    xproj = x;
elseif x_r <= -x_n/mu
    % x is in the polar cone
    xproj = zeros(size(x));
else
    pi_n = (x_r*mu + x_n)/(mu^2 + 1);
    xproj = [pi_n; mu*x(2:end)*pi_n/x_r];
end

end