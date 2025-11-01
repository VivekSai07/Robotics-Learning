%% Extended Unicycle (with wheel rotation)
clear; clc; close all;

% Parameters
r = 0.1;
dt = 0.05;
T  = 20;
N  = T/dt;
q = [0; 0; 0; 0];   % [x, y, theta, phi]

u1 = 5;             % wheel angular speed
u2 = 0.4;           % heading rate

% | Control                 | Meaning                          | Behavior                         |
% | :---------------------- | :------------------------------- | :------------------------------- |
% | (u_1 = 5), (u_2 = 0)    | Wheel spinning, no turning       | Straight line                    |
% | (u_1 = 5), (u_2 = 0.4)  | Wheel spinning + heading turning | Moves along a **circle**         |
% | (u_1 = 0), (u_2 = 0.4)  | No forward motion, only turning  | Spins in place                   |
% | (u_1 = 5), (u_2 = -0.4) | Turns right instead of left      | Circle in the opposite direction |

traj = zeros(4, N);

figure;
for k = 1:N
    % Compute qdot
    qdot = [r*cos(q(3)) 0;
            r*sin(q(3)) 0;
            0           1;
            1           0] * [u1; u2];

    q = q + qdot * dt;
    traj(:,k) = q;

    % Visualization
    clf;
    plot(traj(1,1:k), traj(2,1:k), 'b-', 'LineWidth', 1.5); hold on;
    draw_robot_with_wheel(q(1), q(2), q(3), q(4), r);
    axis equal; grid on;
    xlim([-1.5 1.5]); ylim([-1.5 1.5]);
    title(sprintf('Extended Unicycle (t = %.2fs, φ=%.2f rad)', k*dt, q(4)));
    xlabel('x [m]'); ylabel('y [m]');
    drawnow;
end

%% --- Helper function with wheel visualization ---
function draw_robot_with_wheel(x, y, theta, phi, r)
    L = 0.2;   % robot body length
    plot(x, y, 'ko', 'MarkerFaceColor', 'k');
    quiver(x, y, L*cos(theta), L*sin(theta), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
    % Draw rolling wheel (simple circle with rotation indicator)
    ang = linspace(0, 2*pi, 40);
    wheel_x = r*cos(ang + phi) + x + 0.1*cos(theta);
    wheel_y = r*sin(ang + phi) + y + 0.1*sin(theta);
    plot(wheel_x, wheel_y, 'k', 'LineWidth', 1);
end
