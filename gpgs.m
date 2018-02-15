function x = gpgs(A, b, nl, nc, mu)
% Attempts to solve:
%   x = argmin 0.5*x'*A*x + b'*x

r_max = 30;

x = zeros(nl+2*nc,1);
for r = 1:r_max
    for i = 1:nl+2*nc
        x(i) = x(i) - (A(i,:)*x + b(i))/A(i,i);
        if (i > nl + nc)
            hi = mu(i-nl-nc)*x(i - nc);
            lo = -hi;
            x(i) = min(max(lo, x(i)), hi);
%             x([i-nc,i]) = project(x([i-nc,i]),mu(i-nl-nc));
        else
            x(i) = max(0, x(i));
        end
    end
end

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