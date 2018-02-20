function [v_next, x] = solver_blcp(v_prev, Fext, M, J, mu, psi, h)
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
c = J*(v_prev + M\Fext*h);

% Baumgarte stabilization
b = c + [psi/h; zeros(2*nc,1)];

%% Linear Complementarity Problem with Bounds (BLCP)
D = diag(A);

% Solve for contact impulses (Projected Gauss-Seidel)
x = zeros(3*nc,1);
for r = 1:30
    for i = 1:nc
        % Normal
        xnew = x(i) - (A(i,:)*x + b(i))/D(i);
        x(i) = max(0, xnew(1));
        lim = mu(i)*x(i);

        % Tangent 1
        xnew = x(i+1*nc) - (A(i+1*nc,:)*x + b(i+1*nc))/D(i+1*nc);
        x(i+1*nc) = min(max(-lim, xnew), lim);

        % Tangent 2
        xnew = x(i+2*nc) - (A(i+2*nc,:)*x + b(i+2*nc))/D(i+2*nc);
        x(i+2*nc) = min(max(-lim, xnew), lim);
    end
end

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

end