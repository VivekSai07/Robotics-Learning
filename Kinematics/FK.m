clear; clc; close all;

% Link lengths
L1 = 1.0;
L2 = 0.8;

% Joint angles (in radians)
theta1 = deg2rad(45);
theta2 = deg2rad(30);

% Base point
x0 = 0; y0 = 0;

% First joint
x1 = L1*cos(theta1);
y1 = L1*sin(theta1);

% End-effector
x2 = x1 + L2*cos(theta1 + theta2);
y2 = y1 + L2*sin(theta1 + theta2);


figure;
plot([x0 x1 x2], [y0 y1 y2], '-o', 'LineWidth', 2, 'MarkerSize', 8);
axis equal;
grid on;
xlim([-2 2]);
ylim([-2 2]);
xlabel('X');
ylabel('Y');
title('2-Link Planar Manipulator');