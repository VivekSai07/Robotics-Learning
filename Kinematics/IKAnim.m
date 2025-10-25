%% Inverse Kinematics Animation - 2-Link Planar Robot
clear; clc; close all;

% --- Link lengths ---
L1 = 1.0;
L2 = 0.8;

% --- Path for End-Effector (circle path) ---
t_vals = linspace(0, 2*pi, 200);
radius = 1.2;
center = [0.2, 0.2];
x_target = center(1) + radius*cos(t_vals);
y_target = center(2) + radius*sin(t_vals);

% --- Figure setup ---
figure('Color','w');
hold on; grid on; axis equal;
xlim([-2 2]); ylim([-2 2]);
xlabel('X'); ylabel('Y');
title('2-Link Planar Robot - Inverse Kinematics Animation');

% --- Initialize graphics objects ---
armLine = plot([0 0 0], [0 0 0], '-o', 'LineWidth', 2, 'MarkerSize', 8);
targetPoint = plot(0,0,'rx','MarkerSize',10,'LineWidth',2);
textInfo = text(-1, -1, '', 'FontSize', 12, 'FontWeight', 'bold', 'FontName','Consolas', 'BackgroundColor',[0.95 0.95 0.95]);

% --- Animation loop ---
for k = 1:length(t_vals)
    % Desired End-effector position
    xd = x_target(k);
    yd = y_target(k);

    % --- Inverse Kinematics (Analytical for 2-link planar arm) ---
    D = (xd^2 + yd^2 - L1^2 - L2^2) / (2*L1*L2);
    % Clamp to avoid numerical issues
    D = max(min(D,1),-1);

    % Elbow-down solution
    theta2 = atan2(-sqrt(1 - D^2), D);
    theta1 = atan2(yd, xd) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));

    % --- Forward Kinematics for plotting ---
    x1 = L1*cos(theta1);
    y1 = L1*sin(theta1);
    x2 = x1 + L2*cos(theta1 + theta2);
    y2 = y1 + L2*sin(theta1 + theta2);

    % --- Update graphics ---
    set(armLine, 'XData', [0 x1 x2], 'YData', [0 y1 y2]);
    set(targetPoint, 'XData', xd, 'YData', yd);

    % --- Update info text ---
    msg = sprintf(['θ₁ = %5.1f°\nθ₂ = %5.1f°\n\n' ...
                   'End-Effector:\nX = %6.3f\nY = %6.3f'], ...
                   rad2deg(theta1), rad2deg(theta2), x2, y2);
    set(textInfo, 'String', msg);

    drawnow;
    pause(0.03);
end
