% Parameters
h = 0.02;
mu = 0.3;
m = 0.2;
r = 0.5;
m_p = 1;
params = struct('h', h, 'mu', mu, 'm', m, 'r', r, 'm_p', m_p);

% Disk
angles = linspace(0, 2*pi, 20);
xs = r*cos(angles);
ys = r*sin(angles);

% Initial pose
x = [0; 0.5; -1; 0; 0; 0];
ax = axes(); hold on;
disk = plot(xs + r*cos(x(1)), ys + r*sin(x(1)), 'k');
targ = plot(x(2), x(3), '+r');
pt = plot(x(2), x(3), 'g.');
axis([-2 2 -2 2])

% PD control
kp = 100;
kd = 20;
err_last = 0;
umax = 10;

% Wait for user to select target
cursor = ax.CurrentPoint(1,1:2);
while (cursor == ax.CurrentPoint(1,1:2))
    pause(0.1)
end

while (true)
    cursor = ax.CurrentPoint(1,1:2);
    
    % PD control
    err = x(2:3) - cursor(:);
    u = -kp*err - kd*(err - err_last)/h;
    u = max(min(u, umax), -umax);
    err_last = err;
    
    % Dynamics
    x = forward_lcp(params, x, u);
    
    % Plotting
    disk.XData = xs + r*cos(x(1));
    disk.YData = ys + r*sin(x(1));
    targ.XData = cursor(1);
    targ.YData = cursor(2);
    pt.XData = x(2);
    pt.YData = x(3);
    axis([-2 2 -2 2])
    
    pause(h)
end
