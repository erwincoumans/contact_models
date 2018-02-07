function [q_next, v_next, f] = forward_ccp(h, M, q_prev, v_prev, Fext, mu, psi, J)
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

[D1, E1, eta1, gamma1] = deal(cell(nl,1));
for i = 1:nl
    D1{i} = J(i,:)';
    E1{i} = M\D1{i};
    eta1{i} = 1/trace(D1{i}'*E1{i});
    gamma1{i} = 0;
end
[D2, E2, eta2, gamma2] = deal(cell(nc,1));
for i = 1:nc
    D2{i} = J(nl + i + [0,nc],:)';
    E2{i} = M\D2{i};
    eta2{i} = 2/trace(D2{i}'*E2{i});
    gamma2{i} = [0 0]';
end


% ktilde = M*v_prev + Fext*h;
% v{1} = M\ktilde;
v0 = v_prev + M\Fext*h;
lambda = 1;
omega = 1;
r_max = 300;
tol = 1e-6;

v = v0;
for r = 1:r_max
    vprev = v;
    gprev1 = gamma1;
    gprev2 = gamma2;
    for i = 1:nl
        delta = gamma1{i} - omega*eta1{i}*(D1{i}'*v + psi(i)/h);
        gamma1{i} = lambda*max(delta, 0) + (1-lambda)*gamma1{i};
        v = v + E1{i}*(gamma1{i} - gprev1{i});
    end
    for i = 1:nc
        delta = gamma2{i} - omega*eta2{i}*(D2{i}'*v + [psi(nl + i)/h; 0]);
        gamma2{i} = lambda*project(delta, mu(i)) + (1-lambda)*gamma2{i};
        v = v + E2{i}*(gamma2{i} - gprev2{i});
    end
    if all(abs(v - vprev) < tol)
        break
    end
end
if (r == r_max)
    disp('Max iterations reached')
end

v_next = v;
f = [gamma2{:}];
f = cat(1, gamma1{:}, f(:));
f = [f(1:2:end); f(2:2:end)];
q_next = q_prev + h*v_next;

end

function pi_g = project(gamma, mu)

g_n = gamma(1);
g_r = norm(gamma(2:end));

if g_r <= mu*g_n
    pi_g = gamma;
elseif g_r <= -g_n/mu
    pi_g = zeros(size(gamma));
else
    pi_n = (g_r*mu + g_n)/(mu^2 + 1);
    pi_g = [pi_n; mu*gamma(2:end)*pi_n/g_r];
end

end