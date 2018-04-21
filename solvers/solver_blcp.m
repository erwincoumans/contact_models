function [v_next, x] = solver_blcp(v_prev, Fext, M, J, mu, psi, h)
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

%% Bounded Linear Complementarity Problem (BLCP)
d = diag(A);

% Solve for contact impulses (Projected Gauss-Seidel)
x = zeros(3*nc,1);
for r = 1:30
    for i = 1:nc
        % Normal (n)
        xnew = x(i) - (A(i,:)*x + btilde(i))/d(i);
        x(i) = max(0, xnew);
        lim = mu(i)*x(i);

        % Tangent (t)
        xnew = x(i+1*nc) - (A(i+1*nc,:)*x + btilde(i+1*nc))/d(i+1*nc);
        x(i+1*nc) = min(max(-lim, xnew), lim);

        % Tangent (o)
        xnew = x(i+2*nc) - (A(i+2*nc,:)*x + btilde(i+2*nc))/d(i+2*nc);
        x(i+2*nc) = min(max(-lim, xnew), lim);
    end
end

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

end