function [q_next, v_next, f] = forward_lcp(h, M, q_prev, v_prev, Fext, mu, psi, J)
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

% Exta tangential impulse variables (reversed sign)
J = [J; -J(nl+nc+1:end,:)];
E = [eye(nc); eye(nc)];

% Helper variables
U = diag(mu); % [nc x nc]
A = J*(M\J'); % [nl+2*nc x nl+2*nc]
A = (A + A')/2; % should be symmetric
c = J*(v_prev + M\Fext*h); % [nl+2*nc x 1]

% Contact smoothing
cfm = 1e-6;
A = (A + diag([(cfm/h)*ones(nl+nc,1); zeros(2*nc,1)]));

% Constraints
Mat = [A [zeros(nl+nc,nc); E]
     zeros(nc, nl) U -E' zeros(nc)];
vec = [c + [psi/h; zeros(2*nc,1)]; zeros(nc,1)];

% Solve for contact impulses
z = lemke(Mat, vec);
f = z(1:nl+2*nc);
f(nl+nc+(1:nc)) = f(nl+nc+(1:nc)) - z(nl+2*nc+(1:nc));

% Calculate next state from contact inpulses
v_next = v_prev + M\(J'*z(1:nl+3*nc) + Fext*h);
q_next = q_prev + h*v_next;

end