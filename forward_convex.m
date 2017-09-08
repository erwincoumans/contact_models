function [q_next, v_next] = forward_convex(h, M, q_prev, v_prev, Fext, mu, psi, J)
% Input:
% h - time step
% M - inertia matrix [n x n]
% q_prev - pose [n x 1]
% v_prev - velocity [n x 1]
% Fext - gravitational and other forces [n x 1]
% mu - coefficients of friction [nc x 1]
% psi - contact normal distances [nl+nc x 1]
% J - contact Jacobian [nl+2*nc x n ] ... assume 2D

% Matrix dimensions
nc = size(mu,1);
nl = size(psi,1) - nc;

% Helper variables
U = diag(mu); % [nc x nc]
A = J*(M\J'); % [nl+2*nc x nl+2*nc]
A = (A + A')/2; % should be symmetric
c = J*(v_prev + M\Fext*h); % [nl+2*nc x 1]

% Contact smoothing
Rmax = 100;
Rmin = 0.01;
wmax = 0.1;
psi_rep = psi([1:end,nl+1:end]); % [nl+2*nc x 1]
R = diag((Rmin + (Rmax - Rmin)*psi_rep/wmax));

% Constraints
Alt = [-A(1:nl+nc,:);... % no penetration
    zeros(nc,nl) -U +eye(nc);... % friction cone
    zeros(nc,nl) -U -eye(nc);... % friction cone
    -eye(nl+nc) zeros(nl+nc,nc)]; % no attractive contact forces
blt = [c(1:nl+nc,:) + psi/h; zeros(nl+3*nc,1)];

% Solve for contact impulses
f = quadprog(A + R, c, Alt, blt, [], [], [], [], [], ...
    optimset('Algorithm', 'interior-point-convex', 'Display', 'off'));
% f = primal_interior_point(A+R, c, -Alt, -blt, nl, nc);

% Calculate next state from contact inpulses
v_next = v_prev + M\(J'*f + Fext*h);
q_next = q_prev + h*v_next;

end