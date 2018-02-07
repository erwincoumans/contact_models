function x = pgs(A,b)
% Solves 0 <= x _|_ Ax+b >= 0

tol = 1e-6;
r_max = 1000;

A = A + 5e-5*eye(size(A)); % cfm

x = zeros(size(A,2),1);
x_next = x;
for r = 1:r_max
    for i = 1:size(x_next,1)
        y = A(i,:)*x_next + b(i);
        x_next(i) = max(0, x_next(i) - y/A(i,i));
    end
    if all(abs(x_next - x) < tol)
        break
    end
    x = x_next;
end
if (r == r_max)
    disp('Max iterations reached')
end

end