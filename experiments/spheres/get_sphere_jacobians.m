function [psi, J] = get_sphere_jacobians(q, v, h, r, nx, n)
% Gap distances and Jacobians

ctol = 0.1;

psi = cell(1, n*(n-1)/2 + 3*n);
J = cell(1, n*(n-1)/2 + 3*n);
nc = 1;

for i = 1:n
    for j = (i+1):n
        dij =    q(7*(j-1)+(1:3)) - q(7*(i-1)+(1:3));
        vij = h*(v(6*(j-1)+(1:3)) - v(6*(i-1)+(1:3)));
        t = max(0, min(1, dot(-dij, vij)/(h*norm(vij))));
        if (norm(dij + t*vij) - 2*r < ctol)
            psi{nc} = norm(dij) - 2*r;
            nhat = dij/norm(dij);
            if abs(nhat(1)) > abs(nhat(2))
                that = cross(nhat, [0; 1; 0]);
            else
                that = cross(nhat, [1; 0; 0]);
            end
            that = that/norm(that);
            ohat = cross(nhat, that);
            G = zeros(3, 6*n);
            rxf = [zeros(3,1) r*cross(that, nhat) r*cross(ohat, nhat)]';
            G(:, 6*(j-1)+(1:3)) = +[nhat that ohat]';
            G(:, 6*(i-1)+(1:3)) = -[nhat that ohat]';
            G(:, 6*(j-1)+(4:6)) = rxf*quat2rotm(q(7*(j-1)+(4:7))');
            G(:, 6*(i-1)+(4:6)) = rxf*quat2rotm(q(7*(i-1)+(4:7))');
            J{nc} = G;
            nc = nc + 1;
        end
    end
end
for i = 1:n
    x = q(7*(i-1)+(1:3));
    dx = q(6*(i-1)+(1:3));
    R = quat2rotm(q(7*(i-1)+(4:7))');
    t = max(0, min(1, (r - x(1))/dx(1)));
    if (abs(x(1) + t*dx(1) - r) < ctol)
        psi{nc} = x(1) - r;
        G = zeros(3, 6*n);
        G(:, 6*(i-1)+(1:3)) = [ 1  0  0;  0  1  0;  0  0  1];
        G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0  0 -r;  0  r  0]*R;
        J{nc} = G;
        nc = nc + 1;
    end
    t = max(0, min(1, (r - x(2))/dx(2)));
    if (abs(x(2) + t*dx(2) - r) < ctol)
        psi{nc} = x(2) - r;
        G = zeros(3, 6*n);
        G(:, 6*(i-1)+(1:3)) = [ 0  1  0; -1  0  0;  0  0  1];
        G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0  0 -r; -r  0  0]*R;
        J{nc} = G;
        nc = nc + 1;
    end
    t = max(0, min(1, (r - x(3))/dx(3)));
    if (abs(x(3) + t*dx(3) - r) < ctol)
        psi{nc} = x(3) - r;
        G = zeros(3, 6*n);
        G(:, 6*(i-1)+(1:3)) = [ 0  0  1;  1  0  0;  0  1  0];
        G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0 -r  0;  r  0  0]*R;
        J{nc} = G;
        nc = nc + 1;
    end
    t = max(0, min(1, ((3*nx*r - r) - x(1))/dx(1)));
    if (abs(x(1) + t*dx(1) - (3*nx*r - r)) < ctol)
        psi{nc} = 3*nx*r - r - x(1);
        G = zeros(3, 6*n);
        G(:, 6*(i-1)+(1:3)) = [-1  0  0;  0 -1  0;  0  0  1];
        G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0  0 -r;  0 -r  0]*R;
        J{nc} = G;
        nc = nc + 1;
    end
    t = max(0, min(1, (3*nx*r - r - x(2))/dx(2)));
    if (abs(x(2) + t*dx(2) - (3*nx*r - r)) < ctol)
        psi{nc} = 3*nx*r - r - x(2);
        G = zeros(3, 6*n);
        G(:, 6*(i-1)+(1:3)) = [ 0 -1  0;  1  0  0;  0  0  1];
        G(:, 6*(i-1)+(4:6)) = [ 0  0  0;  0  0 -r;  r  0  0]*R;
        J{nc} = G;
        nc = nc + 1;
    end
end

psi = cat(1, psi{:});
J = cat(1, J{:});
J = cat(1, J(1:3:end,:), J(2:3:end,:), J(3:3:end,:));
end