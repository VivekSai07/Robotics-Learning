function xplus = ddmodel(x, b)
% x = [x, y, theta, sL, sR]
% returns the updated pose [x+, y+, theta+]
    
    % Extract state variables
    xp = x(1);
    yp = x(2);
    th = x(3);
    sL = x(4);
    sR = x(5);

    % Compute delta theta
    dth = (sR - sL) / b;

    % Check straight motion
    if abs(dth) < 1e-9
        % Straight
        s = 0.5 * (sL + sR);
        xp_new = xp + s * cos(th);
        yp_new = yp + s * sin(th);
        th_new = th;
    else
        % Arc motion
        R = (b/2) * (sL + sR) / (sR - sL);

        xp_new = xp + R * (sin(th + dth) - sin(th));
        yp_new = yp - R * (cos(th + dth) - cos(th));
        th_new = th + dth;
    end

    % Final state (pose only)
    xplus = [xp_new; yp_new; th_new];
end
