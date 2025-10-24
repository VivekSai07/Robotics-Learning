% Nullspace Visualization with Input (x) and Output (A*x)
clear; clc; close all;

% Define a 2x2 matrix (rank-deficient)
A = [2 4; 1 2];  % You can modify this

% Compute nullspace
N = null(A);  % orthonormal basis for nullspace
ns_dir = N(:,1) / norm(N(:,1));

% Display matrix and nullspace
disp('Matrix A = '); disp(A);
disp('Nullspace basis vector = '); disp(ns_dir);

% Setup figure with two panels
figure('Color','w');
tiledlayout(1,2, 'Padding','compact', 'TileSpacing','compact');

% Animation setup
t_vals = linspace(-3, 3, 100);

for k = 1:length(t_vals)
    x = t_vals(k) * ns_dir;   % current input vector along nullspace
    Ax = A * x;               % corresponding output vector
    
    % ======= INPUT SPACE (x) =======
    nexttile(1);
    cla; hold on; axis equal;
    axis([-4 4 -4 4]);
    grid on;
    xlabel('x_1'); ylabel('x_2');
    title('Input Space: x');
    plot([-4 4],[0 0],'k--','LineWidth',0.5);
    plot([0 0],[-4 4],'k--','LineWidth',0.5);
    plot([-4 4]*ns_dir(1), [-4 4]*ns_dir(2), 'g', 'LineWidth',2);
    quiver(0,0,x(1),x(2),0,'b','LineWidth',2,'MaxHeadSize',0.5);
    legend('Nullspace','Vector x','Location','southoutside');
    
    % ======= OUTPUT SPACE (A*x) =======
    nexttile(2);
    cla; hold on; axis equal;
    axis([-4 4 -4 4]);
    grid on;
    xlabel('y_1'); ylabel('y_2');
    title('Output Space: A * x');
    plot([-4 4],[0 0],'k--','LineWidth',0.5);
    plot([0 0],[-4 4],'k--','LineWidth',0.5);
    
    % Draw output vector A*x
    quiver(0,0,Ax(1),Ax(2),0,'r','LineWidth',2,'MaxHeadSize',0.5);
    
    % Display info box
    text(-3.5,3.2,sprintf('A = [%.1f %.1f; %.1f %.1f]',A(1,1),A(1,2),A(2,1),A(2,2)), ...
        'BackgroundColor',[0.8 0.95 1]);
    text(-3.5,2.6,sprintf('x = [%.2f; %.2f]',x(1),x(2)), ...
        'BackgroundColor',[0.9 1 0.9]);
    text(-3.5,2.0,sprintf('A*x = [%.2f; %.2f]',Ax(1),Ax(2)), ...
        'BackgroundColor',[1 0.9 0.9]);
    
    drawnow;
    pause(0.05);
end