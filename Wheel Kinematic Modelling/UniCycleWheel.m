%% Basic Unicycle Kinematics Visualization
clear; clc; close all;

% Parameters
r = 0.1;             % wheel radius [m]
dt = 0.05;           % time step [s]
T  = 20;             % total time [s]
N  = T/dt;

% Initial state [x, y, theta]
q = [0; 0; 0];

% Controls (u1 = wheel angular velocity, u2 = heading rate)
u1 = 5;              % rad/s -> forward rolling speed
u2 = 0;            % rad/s -> turning speed

% | Control                 | Meaning                          | Behavior                         |
% | :---------------------- | :------------------------------- | :------------------------------- |
% | (u_1 = 5), (u_2 = 0)    | Wheel spinning, no turning       | Straight line                    |
% | (u_1 = 5), (u_2 = 0.4)  | Wheel spinning + heading turning | Moves along a **circle**         |
% | (u_1 = 0), (u_2 = 0.4)  | No forward motion, only turning  | Spins in place                   |
% | (u_1 = 5), (u_2 = -0.4) | Turns right instead of left      | Circle in the opposite direction |

% Storage for trajectory
traj = zeros(3, N);

figure;
for k = 1:N
    % Compute derivatives
    qdot = [r*cos(q(3)) 0;
            r*sin(q(3)) 0;
            0           1] * [u1; u2];
    
    % Integrate
    q = q + qdot * dt;
    traj(:,k) = q;

    % Visualization
    clf;
    plot(traj(1,1:k), traj(2,1:k), 'b-', 'LineWidth', 1.5); hold on;
    draw_robot(q(1), q(2), q(3));
    axis equal; grid on;
    xlim([-1.5 1.5]); ylim([-1.5 1.5]);
    title(sprintf('Basic Unicycle Model (t = %.2fs)', k*dt));
    xlabel('x [m]'); ylabel('y [m]');
    drawnow;
end

%% --- Helper function to draw robot ---
function draw_robot(x, y, theta)
    L = 0.2;  % arrow length
    plot(x, y, 'ko', 'MarkerFaceColor', 'k');
    quiver(x, y, L*cos(theta), L*sin(theta), 'r', 'LineWidth', 2, 'MaxHeadSize', 2);
end
