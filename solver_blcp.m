function [q_next, v_next, x] = solver_blcp(q_prev, v_prev, Fext, M, J, mu, psi, h)
% Input:
%   q_prev - pose [n x 1]
%   v_prev - velocity [n x 1]
%   Fext - gravitational and other forces [n x 1]
%   M - inertia matrix [n x n]
%   J - contact Jacobian [2*nc x n]
%   mu - coefficients of friction [nc x 1]
%   psi - contact gap distances [nc x 1]
%   h - time step
% Output:
%   q_next - pose [n x 1]
%   v_prev - velocity [n x 1]
%   x - contact impulses [nc x 1]

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
x = zeros(2*nc,1);
for r = 1:30
    for i = 1:2*nc
        % Single element update
        xnew = x(i) - (A(i,:)*x + b(i))/D(i);
        % Project impulse into friction cone
        if (i > 2*nc)
            % tangential direction 2
            x_n = mu(i - 2*nc)*x(i - 2*nc);
            x(i) = min(max(-x_n, xnew), x_n);
        elseif (i > nc)
            % tangential direction 1
            x_n = mu(i - nc)*x(i - nc);
            x(i) = min(max(-x_n, xnew), x_n);
        else
            % normal direction
            x(i) = max(0, xnew);
        end
    end
end

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);
q_next = q_prev + h*v_next;

end