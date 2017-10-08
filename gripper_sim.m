function [x, f] = gripper_sim(params, x0, u)
N = size(u,2) + 1;
x = zeros(size(x0,1), N);
x(:,1) = x0;
f = cell(1,N);
for k = 2:N
    [x(:,k), f{k-1}] = gripper_step(params, x(:,k-1), u(:,k-1));
end
f = [f{:}];
end