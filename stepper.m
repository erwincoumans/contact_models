function [x, f] = stepper(params, sys_fun, x0, u, N)
x = zeros(size(x0,1), N);
x(:,1) = x0;
f = cell(1,N);
if (size(u, 2) == 1)
    for k = 2:N
        [x(:,k), f{k}] = sys_fun(params, x(:,k-1), u);
    end
else
    for k = 2:N
        [x(:,k), f{k}] = sys_fun(params, x(:,k-1), u(:,k-1));
    end
end
f{1} = zeros(size(f{2}));
f = [f{:}];
end