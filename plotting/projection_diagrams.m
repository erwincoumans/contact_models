function projection_diagrams
% Illustrate different projection procedures.
clf

p = [-1; 1; 1.5]; % initial impulse (p = x^*)
mu = 1/3;

%% Cone (NCP)
s(1) = subplot(1,3,1);

% Cone
[x,y,z] = cylinder(1,50);
x(1,:) = 0;
y(1,:) = 0;
z = 3*z;
surf(x,y,z,'LineStyle','none','FaceAlpha',0.3,'FaceColor',[0 0 1])
hold on
plot3(x(2,:),y(2,:),z(2,:),'-k')

% Impulse and projection
quiver3(0,0,0,p(1),p(2),p(3),'.r','LineWidth',2,'AutoScale','off');
pp = p;
pp(1:2) = project_circle(pp(1:2), pp(3)*mu);
quiver3(0,0,0,pp(1),pp(2),pp(3),'.r','LineWidth',2,'AutoScale','off');
plot3([p(1) pp(1)],[p(2) pp(2)],[p(3) pp(3)],':r','LineWidth',1)
hold off

% View
axis off
axis([-1.5 1.5 -1.5 1.5 0 3])
axis equal
view(135,20)
title('NCP')

%% Pyramid (BLCP)
s(2) = subplot(1,3,2);

% Pyramid
z = 3*[0 0 0 0 0; 1 1 1 1 1];
x = [0 0 0 0 0; 1 1 -1 -1 1];
y = [0 0 0 0 0; 1 -1 -1 1 1];
surf(x,y,z,'LineStyle','-','FaceAlpha',0.3,'FaceColor',[0 0 1])
hold on

% Impulse and projections
quiver3(0,0,0,p(1),p(2),p(3),'.r','LineWidth',2,'AutoScale','off');
pc = p;
pc(2) = min(max(-p(3)*mu, p(2)), p(3)*mu);
pp = pc;
pp(1) = min(max(-p(3)*mu, p(1)), p(3)*mu);
quiver3(0,0,0,pc(1),pc(2),pc(3),'.r','LineWidth',2,'AutoScale','off');
quiver3(0,0,0,pp(1),pp(2),pp(3),'.r','LineWidth',2,'AutoScale','off');
plot3([p(1) pc(1)],[p(2) pc(2)],[p(3) pc(3)],':r','LineWidth',1)
plot3([pc(1) pp(1)],[pc(2) pp(2)],[pc(3) pp(3)],':r','LineWidth',1)
hold off

% View
axis off
axis([-1.5 1.5 -1.5 1.5 0 3])
axis equal
view(132,20)
title('BLCP')

%% Cone (CCP)
s(3) = subplot(1,3,3);

% Cone
[x,y,z] = cylinder(1,50);
x(1,:) = 0;
y(1,:) = 0;
z = 3*z;
surf(x,y,z,'LineStyle','none','FaceAlpha',0.3,'FaceColor',[0 0 1])
hold on
plot3(x(2,:),y(2,:),z(2,:),'-k')

% Impulse and projection
quiver3(0,0,0,p(1),p(2),p(3),'.r','LineWidth',2,'AutoScale','off');
pp = project_cone(p,mu);
quiver3(0,0,0,pp(1),pp(2),pp(3),'.r','LineWidth',2,'AutoScale','off');
plot3([p(1) pp(1)],[p(2) pp(2)],[p(3) pp(3)],':r','LineWidth',1)
hold off

% View
axis off
axis([-1.5 1.5 -1.5 1.5 0 3])
axis equal
view(135,20)
title('CCP')

%% Formatting
% Plot sizes
set(gcf,'Position',[100 100 550 250])
s(1).Position = [0/3 0 1/3-0.05 0.85];
s(2).Position = [1/3 0 1/3-0.05 0.85];
s(3).Position = [2/3 0 1/3-0.05 0.85];

% Annotations
annotation('textbox',[0.26 0.5 0.1 0.1],'Interpreter','latex',...
    'LineStyle','None','FontSize',22,'String','$x^*$','Color','r');
annotation('textbox',[0.59 0.5 0.1 0.1],'Interpreter','latex',...
    'LineStyle','None','FontSize',22,'String','$x^*$','Color','r');
annotation('textbox',[0.93 0.5 0.1 0.1],'Interpreter','latex',...
    'LineStyle','None','FontSize',22,'String','$x^*$','Color','r');

end

function xproj = project_cone(x, mu)
% Project impulse into friction cone

x_n = x(3); % normal
x_f = norm(x(1:2)); % combined frictional

if x_f <= mu*x_n
    % x is already in the cone
    xproj = x;
elseif x_f <= -x_n/mu
    % x is in the polar cone
    xproj = zeros(size(x));
else
    pi_n = (x_f*mu + x_n)/(mu^2 + 1);
    xproj = [mu*x(1:2)*pi_n/x_f; pi_n];
end
end

function xproj = project_circle(x, r)
% Project frictional impulse into a circle of radius r

x_f = norm(x); % combined frictional

if x_f <= r
    xproj = x;
else
    xproj = x*(r/x_f);
end
end
