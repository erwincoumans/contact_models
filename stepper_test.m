function [dm, x1, x2, f1, f2] = stepper_test(params, sys_fun, x0, u, N)
params.h = 0.01;
params2 = params;
params2.h = 0.001;

[x1,x2,f1,f2] = deal(cell(1,N));

x1{1} = x0;
x2{1} = x0;
for i = 2:N
    x1{i} = x1{i-1};
    x2{i} = x2{i-1};
    [x1{i}, f1{i}] = sys_fun(params, x1{i}, u(x1{i}));
    for j = 1:10
        [x2{i}, f2{i}] = sys_fun(params2, x2{i}, u(x2{i}));
    end
end
f1{1} = zeros(size(f1{2}));
f2{1} = zeros(size(f2{2}));

x1 = [x1{:}];
x2 = [x2{:}];
f1 = [f1{:}];
f2 = [f2{:}];
dm = (max(abs(x1(1:3,:) - x2(1:3,:))));
end