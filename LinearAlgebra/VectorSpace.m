% Vector Space Visualization (Span of Vectors)
clear; clc; close all;

% Define base vectors (you can modify them)
x1 = [2; 1];
x2 = [-1; 2];

% Create a grid of coefficients c1, c2
[c1, c2] = meshgrid(linspace(-2, 2, 21), linspace(-2, 2, 21));

% Compute all linear combinations
X = c1 * x1(1) + c2 * x2(1);
Y = c1 * x1(2) + c2 * x2(2);

% Setup figure
figure('Color','w');
axis equal
axis([-5 5 -5 5]);
hold on;
grid on;
xlabel('x_1 axis');
ylabel('x_2 axis');
title('Visualization of Vector Space: span({x_1, x_2})');

% Plot basis vectors
quiver(0,0,x1(1),x1(2),0,'r','LineWidth',2,'MaxHeadSize',0.5);
text(x1(1)*1.1, x1(2)*1.1, 'x_1','FontWeight','bold','Color','r');

quiver(0,0,x2(1),x2(2),0,'b','LineWidth',2,'MaxHeadSize',0.5);
text(x2(1)*1.1, x2(2)*1.1, 'x_2','FontWeight','bold','Color','b');

% Plot the filled region representing the span
fill3(X(:), Y(:), zeros(size(X(:))), 'g', 'FaceAlpha', 0.2, 'EdgeColor','none');

% Animated vector showing linear combinations
h_vec = quiver(0,0,0,0,0,'k','LineWidth',2,'MaxHeadSize',0.5);

% Animate movement through span
t = linspace(0, 2*pi, 200);
for k = 1:length(t)
    c1_t = 1.5*cos(t(k));
    c2_t = 1.5*sin(t(k));
    
    % Compute current linear combination
    vec = c1_t * x1 + c2_t * x2;
    
    % Update the vector
    set(h_vec, 'UData', vec(1), 'VData', vec(2));
    
    % Update text showing coefficients
    title(sprintf('span({x_1, x_2}) : v = %.2f*x_1 + %.2f*x_2', c1_t, c2_t));
    
    pause(0.05);
end