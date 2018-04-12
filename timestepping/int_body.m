function q_next = int_body(q, v, h)
% Input:
%	q - pose [x y z q0 q1 q2 q3]'
%	v - veloctiy [vx vy vz wx wy wz]'
%	h - time step
% Output:
%	q_next - pose

u = norm(v(4:6));
if (u < 1e-8)
    p = [1 0 0 0];
else
    uhat = v(4:6)'/u;
    p = [cos(u*h/2) sin(u*h/2)*uhat];
end

q_next = q;
q_next(1:3) = q(1:3) + h*v(1:3);
q_next(4:7) = quatmultiply(p, q(4:7)')';
end