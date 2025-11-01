%% Car-Like (Ackermann) Robot Visualization
clear; clc; close all;

% Parameters
L = 0.4;        % wheelbase [m]
r = 0.05;       % wheel radius [m]
dt = 0.05;      % timestep [s]
T  = 20;        % total simulation time [s]
N  = T/dt;

% Initial state [x, y, theta, psi]
q = [0; 0; 0; 0];

% Controls: [v; w_s]
v = 0.4;        % forward speed [m/s]
w_s = 0.2;      % steering angle rate [rad/s]

% v = 0.4; w_s = 0;       % straight line
% v = 0.4; w_s = 0.2;     % steer left slowly
% v = 0.4; w_s = -0.2;    % steer right
% v = 0.4; w_s = 0.5*sin(0.1*k*dt); % sinusoidal steering

% Storage
traj = zeros(3, N);

figure;
for k = 1:N
    % --- Dynamics ---
    qdot = [cos(q(3)) 0;
            sin(q(3)) 0;
            tan(q(4))/L 0;
            0 1] * [v; w_s];
    
    q = q + qdot * dt;
    traj(:,k) = q(1:3);
    
    % --- Visualization ---
    clf;
    plot(traj(1,1:k), traj(2,1:k), 'b-', 'LineWidth', 1.5); hold on;
    draw_car(q(1), q(2), q(3), q(4), L, r);
    axis equal; grid on;
    xlim([-1.5 1.5]); ylim([-1.5 1.5]);
    xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Car-like Robot (t=%.2fs, ψ=%.2f°)', k*dt, rad2deg(q(4))));
    drawnow;
end

%% --- Helper function ---
function draw_car(x, y, theta, psi, L, r)
    % Basic body geometry
    body_L = L; body_W = 0.2;
    % corners in body frame
    body = [ body_L/2 -body_L/2 -body_L/2  body_L/2 body_L/2;
             body_W/2  body_W/2 -body_W/2 -body_W/2 body_W/2];
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    B = R*body + [x; y];
    fill(B(1,:), B(2,:), [0.8 0.8 0.8], 'EdgeColor','k');

    % rear axle midpoint
    rear = [x; y] + R*[-L/2; 0];
    % front axle midpoint
    front = [x; y] + R*[L/2; 0];

    % Rear wheels (fixed orientation)
    draw_wheel(rear + R*[0;  body_W/2], theta, r);
    draw_wheel(rear + R*[0; -body_W/2], theta, r);

    % Front wheels (steered by ψ)
    draw_wheel(front + R*[0;  body_W/2], theta + psi, r);
    draw_wheel(front + R*[0; -body_W/2], theta + psi, r);

    % Heading arrow
    quiver(x, y, 0.4*cos(theta), 0.4*sin(theta), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
end

function draw_wheel(center, angle, r)
    % wheel as rectangle
    wL = 0.08; wW = 0.03;
    wheel = [ wL/2 -wL/2 -wL/2  wL/2 wL/2;
              wW/2  wW/2 -wW/2 -wW/2 wW/2];
    R = [cos(angle) -sin(angle); sin(angle) cos(angle)];
    w = R*wheel + center;
    fill(w(1,:), w(2,:), 'k');
end
