function plot_tooltip(q, f)
k = 4;
quiver(q(1,k), q(2,k), f(2,k), f(3,k), 'r');
hold on
quiver(q(1,k), q(2,k), 0.01, 0.006, 'g');
hold off
axis equal
grid on
axis([-0.019  0.019  -0.0143  0.0143])
xlabel('Tool X-Position (m)')
ylabel('Tool Y-Position (m)')
a = gca;
for i = 1:numel(a.Children)
    a.Children(i).LineWidth = 3;
    a.Children(i).MaxHeadSize = 0.5;
end
hold on
plot(q(1,k), q(2,k),'k.','MarkerSize',24)
hold off
a.FontSize = 14;
a.FontWeight = 'bold';
end