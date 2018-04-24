function [st, x] = stepper(params, sys_fun, st0, u, N)
% Input:
%   params - system parameters
%   sys_fun - system dynamics integrator (time stepper)
%   st0 - initial state
%   u - control inputs
%   N - number of time steps to simulate
% Output:
%   st - state trajectory
%   x - contact impulses

st = zeros(size(st0,1), N);
st(:,1) = st0;
x = cell(1,N);
if (size(u, 2) == 1)
    for k = 2:N
        [st(:,k), x{k}] = sys_fun(params, st(:,k-1), u);
    end
else
    for k = 2:N
        [st(:,k), x{k}] = sys_fun(params, st(:,k-1), u(:,k-1));
    end
end
x{1} = zeros(size(x{2}));
x = [x{:}];
end