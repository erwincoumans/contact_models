function [q_next, v_next, f] = forward_pgs(h, M, q_prev, v_prev, Fext, mu, psi, J)
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
A = J*(M\J'); % [nl+2*nc x nl+2*nc]
A = (A + A')/2; % should be symmetric
b = J*(v_prev + M\Fext*h); % [nl+2*nc x 1]
b = b + [psi/h; zeros(nc,1)];

% Solve for contact impulses
tol = 1e-6;
r_max = 30;

x = zeros(nl+2*nc,1);
for r = 1:r_max
    x_prev = x;
    for i = 1:nl+2*nc
        x(i) = x(i) - (A(i,:)*x + b(i))/A(i,i);
        if (i > nl + nc)
            hi = mu(i-nl-nc)*x(i - nc);
            lo = -hi;
            x(i) = min(max(lo, x(i)), hi);
        else
            x(i) = max(0, x(i));
        end
    end
    if all(abs(x - x_prev) < tol)
        break
    end
end
if (r == r_max)
    disp('Max iterations reached')
end
f = x;

% Calculate next state from contact inpulses
v_next = v_prev + M\(J'*f + Fext*h);
q_next = q_prev + h*v_next;

end