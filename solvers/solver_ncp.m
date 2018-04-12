function [v_next, x] = solver_ncp(v_prev, Fext, M, J, mu, psi, h)
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

%% Nonlinear Complementarity Problem (NCP)
d = diag(A);
d(nc+1:2*nc) = min(d(nc+1:2*nc), d(2*nc+1:3*nc));

to = [nc, 2*nc]; % tangent indices

% Solve for contact impulses (Proximal Point/Projected Gauss-Seidel)
x = zeros(3*nc,1);
for r = 1:30
    for i = 1:nc
        % Normal
        xnew = x(i) - (A(i,:)*x + btilde(i))/d(i);
        x(i) = max(0, xnew);

        % Frictional
        xnew = x(i+to) - (A(i+to,:)*x + btilde(i+to))/d(i+nc);
        x(i+to) = project_circle(xnew, mu(i)*x(i));
    end
end

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

end


function xproj = project_circle(x, r)
% Project frictional impulse into a circle of radius r

x_f = norm(x); % combined frictional

if x_f <= r
    xproj = x;
else
    xproj = x*(r/x_f);
end
end