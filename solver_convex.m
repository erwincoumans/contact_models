function [v_next, x] = solver_convex(v_prev, Fext, M, J, mu, psi, h)
% Input:
%   v_prev - velocity [n x 1]
%   Fext - gravitational and other forces [n x 1]
%   M - inertia matrix [n x n]
%   J - contact Jacobian [2*nc x n]
%   mu - coefficients of friction [nc x 1]
%   psi - contact gap distances [nc x 1]
%   h - time step
% Output:
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

%% Convex Quadratic Program

% Contact smoothing
Rmax = 100;
Rmin = 0.01;
wmax = 0.1;
R = diag((Rmin + (Rmax - Rmin)*[psi; psi; psi]/wmax));

% Constraints
U = diag(mu);
Ac = [A(1:nc,:);... % no penetration
      eye(nc)  zeros(nc,2*nc)]; % no attractive contact forces
bc = [-b(1:nc); zeros(nc,1)];

% The substitutions A+R=>A and c=>b improve agreement with LCP

% Solve for contact impulses (Interior-point)
x = interior_point(A + R, c, Ac, bc, mu);
% x = quadprog(A + R, c, -Ac, -bc, [], [], [], [], [], ...
%     optimset('Algorithm', 'interior-point-convex', 'Display', 'off'));
% x = sqopt('contact', @(x) (A + R)*x, c, zeros(size(c)), [], [], -Ac, [], -bc);

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

end