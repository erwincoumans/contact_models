function [x,u,cst,iter] = ddp_grasp(params, op, x0, u0)

DYNCST  = @(x,u,i) sys_dyn_cst(params, x,u,false);

% Trajectory visualization callback
if (exist('op','var') && isfield(op,'plot') && (op.plot > 0))
    x1 = repmat(x0,1,size(u0,2)+1);
    time = 0:params.h:params.h*(size(x1,2)-1);
    figure('ToolBar','none','NumberTitle','off','Name','Trajectory',...
        'Position',[120 678 560 420])
    ph = plot(time,x1(2,:),'b',time,x1(4,:)-x1(5,:),'g',time,x1(6,:),'r');
    xlabel('time (sec)'); ylabel('position (m)');
    plotFn = @(x) update_plot(ph, x);
    op.plotFn = plotFn;
end

[x, u, ~, ~, ~, cost, trace]= iLQG(DYNCST, x0, u0, op);
cst = sum(cost);
iter = numel(trace);
end

function update_plot(ph, x)
ph(1).YData = x(2,:);
ph(2).YData = x(4,:)-x(5,:);
ph(3).YData = x(6,:);
end

function y = dyn_fun(params, x,u)

y = NaN(size(x));
for k = find(~isnan(u(1,:)))
    y(:,k) = step_gripper(params, x(:,k), u(:,k));
end
end

function c = cost_fun(params, x, u)

r = params.r;

final = isnan(u(1,:));
u(:,final) = 0;

% control cost
lu = 1e-5*[1 1 1]*u.^2;

% final cost
if any(final)
%    llf = sabs(x(2,final) - (r+0.5), 0.1);
   llf = 400*(x(2,final) - (r+0.03)).^2;
   lf = double(final);
   lf(final) = llf;
else
   lf = 0;
end

% running cost
lx = 0;
lx = lx + 0.05*sabs(x(4,:)-x(5,:), 0.1);
lx = lx + 0.05*sabs(x(4,:)+x(5,:), 0.1);
lx = lx + 2.0*sabs(x(2,:) - (r+0.03), 0.1);
% lx = lx + 1e-3*(x(2,:) - (r+0.5)).^2;

% total cost
c = lu + lx + lf;
end

function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = sys_dyn_cst(params, x, u, full_DDP)
% combine dynamics and cost
% use helper function finite_difference() to compute derivatives

if nargout == 2
    f = dyn_fun(params, x,u);
    c = cost_fun(params, x,u);
else
    % state and control indices
    ix = 1:12;
    iu = 13:15;
    
    % dynamics first derivatives
    xu_dyn  = @(xu) dyn_fun(params, xu(ix,:),xu(iu,:));
    J       = finite_difference(xu_dyn, [x; u], params.fd);
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
    xu_cost = @(xu) cost_fun(params, xu(ix,:),xu(iu,:));
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

function y = sabs(x,p)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = bsxfun(@plus, sqrt(bsxfun(@plus, x.^2, p.^2)), -p);
end

function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 7.6294e-06;%2^-17;
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