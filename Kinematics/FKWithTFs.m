clear; clc; close all;

%% --- Parameters ---
L1 = 1.0; 
L2 = 0.8;
theta1 = deg2rad(30);
theta2 = deg2rad(30);

%% --- Forward Kinematics ---
% Base
x0 = 0; y0 = 0;

% Joint 1
x1 = L1*cos(theta1);
y1 = L1*sin(theta1);

% End-effector
x2 = x1 + L2*cos(theta1 + theta2);
y2 = y1 + L2*sin(theta1 + theta2);

%% --- Plot Setup ---
figure('Color','w');
hold on; grid on; axis equal;
xlim([-2 2]); ylim([-2 2]);
xlabel('X'); ylabel('Y');
title('2-Link Planar Manipulator with Frames');

%% --- Draw Arm ---
plot([x0 x1 x2], [y0 y1 y2], '-o', 'LineWidth', 2, 'MarkerSize', 8);

%% --- Draw Frames ---
scale = 0.3; % axis length for each frame

% Base frame (W)
drawFrame(x0, y0, 0, scale, 'W');

% Frame at Joint 1
drawFrame(x1, y1, theta1, scale, '1');

% End-effector frame
drawFrame(x2, y2, theta1 + theta2, scale, 'E');

%% --- Display FK Equations ---
eqs = sprintf(['x = L₁cos(θ₁) + L₂cos(θ₁+θ₂)\n' ...
               'y = L₁sin(θ₁) + L₂sin(θ₁+θ₂)\n\n' ...
               'x = %.3f\n' ...
               'y = %.3f'], x2, y2);
text(-1.8, -1.3, eqs, 'FontSize', 12, 'FontName','Consolas', 'BackgroundColor',[0.95 0.95 0.95]);

hold off;

%% --- Helper Function ---
function drawFrame(x, y, theta, scale, label)
    % Rotation matrix
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    % Axes directions
    x_axis = R * [scale; 0];
    y_axis = R * [0; scale];
    % Plot arrows
    quiver(x, y, x_axis(1), x_axis(2), 0, 'r', 'LineWidth', 1.5, 'MaxHeadSize', 1);
    quiver(x, y, y_axis(1), y_axis(2), 0, 'b', 'LineWidth', 1.5, 'MaxHeadSize', 1);
    % Label
    text(x + 0.05, y + 0.05, label, 'FontWeight', 'bold', 'FontSize', 10);
end
