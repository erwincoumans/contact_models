%% System parameters
h = 0.01;
mu = 0.3;
m = 0.2;
r = 0.05;
l = 0.01;
m_g = 2.0;

M = diag([m*[1 1 1 (2/5)*r^2 (2/5)*r^2 (2/5)*r^2], ...
    m_g*[1 1 1 (l^2+r^2)/3 (r^2+r^2)/3 (r^2+l^2)/3], ...
    m_g*[1 1 1 (l^2+r^2)/3 (r^2+r^2)/3 (r^2+l^2)/3]]);

N = 101;
time = 0:h:h*(N-1);

xs = cell(1,2);
for kS = 1:2
    q = zeros(7*3, 1);
    q(4:7:end) = 1;
    q(7+1) = -(r + l);
    q(14+1) = (r + l);
    q(3) = 1;
    v = zeros(6*3, 1);

    % Main loop
    qs = cell(1,N);
    qs{1} = q;
    for iter = 2:N
        %% Calculate J, psi, Fext
        % Gravitational, external, and other forces
        Fext = [0; 0; -9.81*m; zeros(3,1); 4; 0; 0; zeros(3,1); -4; 0; 0];
        for i = 1:3
            omega = v(6*(i-1)+(4:6));
            I = M(6*(i-1)+(4:6), 6*(i-1)+(4:6));
            Fext(6*(i-1)+(4:6)) = -cross(omega, I*omega);
        end

        Q1 = q(7+(4:7))';
        Q2 = q(14+(4:7))';
        Rs = quat2rotm(q(4:7)');
        R1 = quat2rotm(Q1);
        R2 = quat2rotm(Q2);

        nto1 = R1*[ 1 0 0; 0  1 0; 0 0 1]';
        nto2 = R2*[-1 0 0; 0 -1 0; 0 0 1]';
        n1 = nto1(:,1);
        n2 = nto2(:,1);

        d1s = q(1:3) - q(7+(1:3));
        d2s = q(1:3) - q(14+(1:3));

        % Contact gap distances
        psi = [dot(d1s, n1) - r - l; dot(d2s, n2) - r - l];

        r1xs = bsxfun(@cross, d1s - dot(d1s, n1)*n1, -nto1);
        r2xs = bsxfun(@cross, d2s - dot(d2s, n2)*n2, -nto2);
        rsx1 = bsxfun(@cross, -r*n1, nto1);
        rsx2 = bsxfun(@cross, -r*n2, nto2);

        J = [zeros(5,7)      eye(5)          zeros(5,6)
             zeros(5,6)      zeros(5,7)      eye(5)
             nto1' rsx1'*Rs -nto1' r1xs'*R1  zeros(3,6)
             nto2' rsx2'*Rs  zeros(3,6)     -nto2' r2xs'*R2];
        J = J([1:10,11,14,12,15,13,16],:);

        th1 = 2*atan2(norm(Q1(2:4)),Q1(1));
        th2 = 2*atan2(norm(Q2(2:4)),Q2(1));
        err1 = [q(7+(2:3));  th1*Q1(2:4)'/(eps+norm(Q1(2:4)))];
        err2 = [q(14+(2:3)); th2*Q2(2:4)'/(eps+norm(Q2(2:4)))];

        %% Setup
        % Inverse inertia matrix in the contact frame
        A = J*(M\J');
        A = (A + A')/2; % should be symmetric

        % Resulting contact velocities if all contact impulses are 0
        c = J*(v + M\Fext*h);

        % Baumgarte stabilization
        b = c + [err1/h; err2/h; psi/h; zeros(4,1)];

        if (kS == 2)
            %% Bounded Linear Complementarity Problem (BLCP)
            D = diag(A);

            % Solve for contact impulses (Projected Gauss-Seidel)
            x = zeros(16,1);
            for kpgs = 1:30
                for i = [1:10 11 13 15 12 14 16]
                    xnew = x(i) - (A(i,:)*x + b(i))/D(i);
                    if (i == 11) || (i == 12)
                        xnew = max(0, xnew);
                    elseif (i == 13) || (i == 14)
                        lim = mu*x(i-2);
                        xnew = min(max(-lim, xnew), lim);
                    elseif (i == 15) || (i == 16)
                        lim = mu*x(i-4);
                        xnew = min(max(-lim, xnew), lim);
                    end
                    x(i) = xnew;
                end
            end
        else
            %% LCP formulation but with joints, which make it a BLCP/mLCP
            % Augment with exta tangent directions and friction cone
            E = [eye(2); eye(2)];
            U = mu*eye(2);
            A2 = [ A                   -A(:,13:16)      [zeros(12,2); E]
                  -A(13:16,:)           A(13:16,13:16)  E
                   [zeros(2,10) U -E'] -E'              zeros(2)];
            b2 = [b; -b(13:16); zeros(2,1)];

            % Solve for contact impulses (Lemke)
            x = pathlcp(A2, b2, [-inf(10,1); zeros(12,1)], inf(22,1));

            % Collapse extra directions
            x = [x(1:12); x(13:16) - x(17:20)];
        end

        %% Integrate velocity and pose
        v = v + M\(J'*x + Fext*h);

        for i = 1:3
            q(7*(i-1)+(1:7)) = int_body(q(7*(i-1)+(1:7)), v(6*(i-1)+(1:6)), h);
        end
        qs{iter} = q;
    end
    qs = [qs{:}];
    xs{kS} = qs;
end

%% Plotting
plot(time,xs{1}(3,:),'-',time,xs{2}(3,:),'-.')
ylim([0.95, 1.02])

legend('mLCP','BLCP')
xlabel('Time (sec)')
ylabel('Sphere Height (m)')
a = gca;
for k = 1:numel(a.Children)
    a.Children(k).LineWidth = 2;
end
a.FontSize = 14;
a.FontWeight = 'bold';