function Fext = get_sphere_forces(v, m, r, n)
% Gravitational, external, and other forces

Fext = repmat(9.81*m*[0; 0; -1; 0; 0; 0], n, 1);
I = m*(2/5)*r^2;
for i = 1:n
    omega = v(6*(i-1)+(4:6));
    Fext(6*(i-1)+(4:6)) = -cross(omega, I*omega);
end
end