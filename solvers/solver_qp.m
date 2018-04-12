function [v_next, x] = solver_qp(v_prev, Fext, M, J, mu, psi, h)
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

% Contact smoothing
Rmax = 100*(0.01/h);
Rmin = 0.01*(0.01/h);
wmax = 0.1;
R = diag(Rmin + (Rmax - Rmin)*[psi; psi; psi]/wmax);
% R = diag((Rmax - Rmin)/1.7183*(exp([psi; psi; psi]/wmax) - 1) + Rmin);

% Constraints
Ac = [A(1:nc,:);... % no penetration
      eye(nc)  zeros(nc,2*nc)]; % no attractive contact forces
bc = [-btilde(1:nc); zeros(nc,1)];

% The substitutions A+R=>A and b=>btilde improve agreement with LCP

% Solve for contact impulses (Interior-point)
x = interior_point(A + R, b, Ac, bc, mu);

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

end