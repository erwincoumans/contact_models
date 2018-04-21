function [v_next, x] = solver_lcp(v_prev, Fext, M, J, mu, psi, h)
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

%% Linear Complementarity Problem (LCP)

% Augment with exta tangent directions and friction cone
E = [eye(nc); eye(nc)];
Abar = [ A             -A(:,nc+1:end)         [zeros(nc); E]
      -A(nc+1:end,:)  A(nc+1:end,nc+1:end)  E
      [diag(mu) -E'] -E'                    zeros(nc)];
bbar = [btilde; -btilde(nc+1:end); zeros(nc,1)];

% Solve for contact impulses (Lemke)
xbar = lemke(Abar, bbar);
% xbar = pathlcp(Abar, bbar);

% Collapse extra directions
x = [xbar(1:nc); xbar(nc+1:3*nc) - xbar(3*nc+1:5*nc)];

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);

end