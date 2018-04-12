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

%% Convex Quadratic Program
d = diag(A);
d = mean(reshape(d, nc, 3), 2);

% Parameters
kappa = 1e-4; % spring-damper time constant
epsilon = 1e-5; % regularization factor
r_min = 1e-7; % minimum regularization

% Spring-damper stabilization
c_prev = J*v_prev;
btilde = b - c_prev + h*(1 + epsilon)*(2*kappa^(-1)*c_prev + kappa^(-2)*[psi; zeros(2*nc,1)]);

% Regularize A
r = max(d*epsilon, r_min);
d = d + r;
A = A + diag(repmat(R, 3, 1));

nto = [0, nc, 2*nc]; % normal and tangent indices

% Solve for contact impulses (Projected Gauss-Seidel)
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
x_f = norm(x(2:end)); % combined frictional

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