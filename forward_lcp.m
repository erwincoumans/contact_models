function [q_next, v_next, x] = forward_lcp(q_prev, v_prev, Fext, M, J, mu, psi, h)
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
b = c + [psi/h; zeros(nc,1)];

%% Linear Complementarity Problem (LCP)

% Augment with exta tangent directions and friction cone
A2 = [ A                   -A(:,nc+1:end)         [zeros(nc); eye(nc)]
      -A(nc+1:end,:)        A(nc+1:end,nc+1:end)  eye(nc)
       [diag(mu) -eye(nc)] -eye(nc)               zeros(nc)];
b2 = [b; -b(nc+1:end); zeros(nc,1)];

% Solve for contact impulses (Lemke)
x2 = lemke(A2, b2);
% x2 = pathlcp(A2, b2);

% Collapse extra directions
x = [x2(1:nc); x2(nc+1:2*nc) - x2(2*nc+1:3*nc)];

%% Integrate velocity and pose
v_next = v_prev + M\(J'*x + Fext*h);
q_next = q_prev + h*v_next;

end