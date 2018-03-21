function x = analytic_tooltip(~, v0, t)

x = zeros(6, numel(t));
x(4,:) = v0;
x(1,:) = v0*t;

end