%% Simple Inverse Kinematics for 2-Link Planar Manipulator
clear; clc; close all;

% Link lengths
L1 = 1.5;
L2 = 1.5;

% Desired end-effector position
xd = 2.0;
yd = 2.0;

% --- Inverse Kinematics ---
D = (xd^2 + yd^2 - L1^2 - L2^2) / (2 * L1 * L2);

if abs(D) > 1
    error('Target point is out of reach!');
end

% Two possible configurations: elbow-up and elbow-down
theta2_elbow_down = atan2(sqrt(1 - D^2), D);
theta2_elbow_up   = atan2(-sqrt(1 - D^2), D);

% Choose elbow-down
theta2 = theta2_elbow_down;

theta1 = atan2(yd, xd) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

% --- Forward Kinematics for visualization ---
x0 = 0; y0 = 0;
x1 = L1 * cos(theta1);
y1 = L1 * sin(theta1);
x2 = x1 + L2 * cos(theta1 + theta2);
y2 = y1 + L2 * sin(theta1 + theta2);

% --- Visualization ---
figure;
hold on; grid on; axis equal;
xlim([-xd*1.2 xd*1.2]); ylim([-yd*1.2 yd*1.2]);
xlabel('X'); ylabel('Y');
title('2-Link Planar Manipulator - Inverse Kinematics');

% Draw links
plot([x0 x1 x2], [y0 y1 y2], '-o', 'LineWidth', 2, 'MarkerSize', 8);

% Draw desired target
plot(xd, yd, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
% text(xd + 0.01, yd-0.2, 'Target', 'FontSize', 10, 'FontName','Consolas', 'BackgroundColor',[0.95 0.95 0.95]);

% Annotate
text(x2 + 0.1, y2, sprintf('End-effector (%.3f, %.3f)', x2, y2), 'FontSize', 10, 'FontName','Consolas', 'BackgroundColor',[0.95 0.95 0.95]);
text(0.5, -0.3, sprintf('\\theta_1 = %.1f°', rad2deg(theta1)), 'FontSize', 10, 'FontName','Consolas', 'BackgroundColor',[0.95 0.95 0.95]);
text(0.5, -0.5, sprintf('\\theta_2 = %.1f°', rad2deg(theta2)), 'FontSize', 10, 'FontName','Consolas', 'BackgroundColor',[0.95 0.95 0.95]);

