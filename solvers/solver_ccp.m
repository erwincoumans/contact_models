function [v_next, x] = solver_ccp(v_prev, Fext, M, J, mu, psi, h)
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

%% Cone Complementarity Problem (CCP)
d = diag(A);
d = mean(reshape(d, nc, 3), 2);

nto = [0, nc, 2*nc]; % normal and tangent indices

% Solve for contact impulses (Block Projected Gauss-Seidel)
x = zeros(3*nc,1);
for r = 1:30
    for i = 1:nc
        % Block update
        xnew = x(i+nto) - (A(i+nto,:)*x + btilde(i+nto))/d(i);
        x(i+nto) = project_cone(xnew, mu(i));
    end
end

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

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