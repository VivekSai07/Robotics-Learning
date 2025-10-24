% Visualizing Linear Transformation and Determinant (Animated)
clear; clc; close all;

% Define the standard basis vectors
e1 = [1; 0];
e2 = [0; 1];

% Define parameter range (e.g., vary a11 and a22)
theta = linspace(0, 2*pi, 100);  % for smooth animation

figure('Color','w');
axis equal
axis([-4 4 -4 4]);
hold on;
grid on;
xlabel('x-axis');
ylabel('y-axis');
title('Linear Transformation and Determinant (Animation)');

for t = 1:length(theta)
    % Define a transformation matrix that changes over time
    A = [cos(theta(t))  0.5*sin(theta(t));
         sin(theta(t))  cos(theta(t))];

    % Transform the basis vectors
    Ae1 = A * e1;
    Ae2 = A * e2;

    % Compute determinant (area scaling factor)
    detA = det(A);

    % Clear previous frame (except axes)
    cla;
    axis equal;
    axis([-4 4 -4 4]);
    grid on;
    hold on;

    % Draw the transformed parallelogram
    fill([0 Ae1(1) Ae1(1)+Ae2(1) Ae2(1)], ...
         [0 Ae1(2) Ae1(2)+Ae2(2) Ae2(2)], ...
         [0.6 1 0.6], 'FaceAlpha',0.5, 'EdgeColor','k');

    % Plot transformed basis vectors
    quiver(0,0,Ae1(1),Ae1(2),0,'r','LineWidth',2,'MaxHeadSize',0.5);
    quiver(0,0,Ae2(1),Ae2(2),0,'b','LineWidth',2,'MaxHeadSize',0.5);

    % Display matrix, determinant, and area info
    text(2.5,3.2,sprintf('A = [%.2f %.2f; %.2f %.2f]',A(1,1),A(1,2),A(2,1),A(2,2)), ...
         'BackgroundColor',[0.8 0.95 1],'FontWeight','bold');
    text(2.5,2.7,sprintf('Determinant: %.2f',detA), ...
         'BackgroundColor',[1 0.9 0.7],'FontWeight','bold');
    text(2.5,2.2,sprintf('Area of Parallelogram: %.2f',abs(detA)), ...
         'BackgroundColor',[1 0.9 0.7],'FontWeight','bold');

    legend('Transformed Area','Transformed e_1','Transformed e_2','Location','southoutside');

    % Pause for animation
    pause(0.05);
end

