function ddp_contact
clc;
close all

% Set full_DDP=true to compute 2nd order derivatives of the 
% dynamics. This will make iterations more expensive, but 
% final convergence will be much faster (quadratic)
full_DDP = false;
r = 0.5;

% set up the optimization problem
DYNCST  = @(x,u,i) sys_dyn_cst(x,u,full_DDP);
T  = 50; % horizon
x0 = [0, r, 0, 2*r, -2*r, 0, zeros(1,6)]'; % initial state
u0 = .1*randn(3,T); % initial controls
% Op.lims  = [-.5 .5;         % wheel angle limits (radians)
%              -2  2];        % acceleration limits (m/s^2)

% === run the optimization!
[x,u]= iLQG(DYNCST, x0, u0);

sys_plot(x);
end

function y = sys_dynamics(x,u)

y = NaN(size(x));
for k = find(~isnan(u(1,:)))
    y(:,k) = gripper_sim(x(:,k), u(:,k));
end
end

function c = sys_cost(x, u)
% cost function for car-parking problem
% sum of 3 terms:
% lu: quadratic cost on controls
% lf: final cost on distance from target parking configuration
% lx: running cost on distance from origin to encourage tight turns

final = isnan(u(1,:));
u(:,final) = 0;

% control cost
lu = 1e-6*[1 1 1]*u.^2;

% final cost
if any(final)
   llf = sabs(x(2,final) - 1, 0.1);
%    llf = (x(2,final) - 1).^2;
   lf = double(final);
   lf(final) = llf;
else
   lf = 0;
end

% running cost
lx = 0;
lx = lx + 1e-2*sabs(x(4,:)-x(5,:), 0.1);
lx = lx + 1e-2*sabs(x(4,:)+x(5,:), 0.1);
lx = lx + 1e-2*sabs(x(2,:) - 1, 0.1);

% total cost
c     = lu + lx + lf;
end

function y = sabs(x,p)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = bsxfun(@plus, sqrt(bsxfun(@plus, x.^2, p.^2)), -p);
end

function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = sys_dyn_cst(x,u,full_DDP)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives

if nargout == 2
    f = sys_dynamics(x,u);
    c = sys_cost(x,u);
else
    % state and control indices
    ix = 1:12;
    iu = 13:15;
    
    % dynamics first derivatives
    xu_dyn  = @(xu) sys_dynamics(xu(ix,:),xu(iu,:));
    J       = finite_difference(xu_dyn, [x; u]);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    % dynamics second derivatives
    if full_DDP
        xu_Jcst = @(xu) finite_difference(xu_dyn, xu);
        JJ      = finite_difference(xu_Jcst, [x; u]);
        JJ      = reshape(JJ, [4 6 size(J)]);
        JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4])); %symmetrize
        fxx     = JJ(:,ix,ix,:);
        fxu     = JJ(:,ix,iu,:);
        fuu     = JJ(:,iu,iu,:);    
    else
        [fxx,fxu,fuu] = deal([]);
    end    
    
    % cost first derivatives
    xu_cost = @(xu) sys_cost(xu(ix,:),xu(iu,:));
    J       = squeeze(finite_difference(xu_cost, [x; u]));
    cx      = J(ix,:);
    cu      = J(iu,:);
    
    % cost second derivatives
    xu_Jcst = @(xu) squeeze(finite_difference(xu_cost, xu));
    JJ      = finite_difference(xu_Jcst, [x; u]);
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
    cxx     = JJ(ix,ix,:);
    cxu     = JJ(ix,iu,:);
    cuu     = JJ(iu,iu,:);
    
    [f,c] = deal([]);
end
end

function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 1e-3;%2^-17;
end

[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = bsxfun(@plus, x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = bsxfun(@plus, Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);
end

function sys_plot(x)
h = 0.02;
r = 0.5;

angles = linspace(0, 2*pi, 30);
xs = r*cos(angles);
ys = r*sin(angles);

lims = [-4 4 -1 5]*r;
figure()
patch(lims([1 2 2 1]), [lims([3 3]) 0 0], 0.8+[0 0 0]);
h_x1 = line(x(4)+[0 0], x(6)+[0 3*r], 'Color', 'k', 'LineWidth', 2);
h_x2 = line(x(5)+[0 0], x(6)+[0 3*r], 'Color', 'k', 'LineWidth', 2);
h_ceil = patch([-2 2 2 -2]*r, x(6)+[3 3 4 4]*r, 'k');
h_disk = patch(x(1) + xs, x(2) + ys, 0.8+[0 0 0]);
h_tick = line(x(1)+[0 r*cos(x(3))], x(2)+[0 r*sin(x(3))], 'Color', 'k', 'LineStyle', '--');
axis(lims)

for k = 1:size(x,2)
    h_x1.XData = x(4,k)+[0 0];
    h_x1.YData = x(6,k)+[0 3*r];
    h_x2.XData = x(5,k)+[0 0];
    h_x2.YData = x(6,k)+[0 3*r];
    h_ceil.YData = x(6,k)+[3 3 4 4]*r;
    h_disk.XData = x(1,k) + xs;
    h_disk.YData = x(2,k) + ys;
    h_tick.XData = x(1,k)+[0 r*cos(x(3,k))];
    h_tick.YData = x(2,k)+[0 r*sin(x(3,k))];
    axis(lims)
    pause(h)
end
end