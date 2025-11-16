clc; clear; close all;

b = 0.5;

cases = {
    [0;0;0;0.3;0.3], "Straight"
    [0;0;0;0.3;0.5], "Curve"
};

figure; hold on; axis equal; grid on;

for i = 1:2
    x = cases{i,1};
    label = cases{i,2};
    pts = zeros(50,2);
    sL = x(4);
    sR = x(5);
    cur = x;  % initial state
    
    for k = 1:50
        cur = [cur(1:3); sL; sR];  % ensure 5 elements
        cur = ddmodel(cur, b);
        pts(k,:) = cur(1:2)';
    end
    
    plot(pts(:,1), pts(:,2), 'LineWidth', 2, 'DisplayName', label);
end

legend
title("Straight vs Curved Motion (ddmodel)");
xlabel("x"); ylabel("y");

%%
clc; clear; close all;

x = [0;0;0;0.3;0.3];   % small increments
b = 0.5;
visualize_arc(x, b, 200);

%%
clc; clear; close all;
b = 0.5;

figure; hold on; axis equal; grid on;
title("Motion field of ddmodel");

x0 = [0; 0; 0];

for sL = -0.3:0.3:0.3
    for sR = -0.3:0.3:0.3
        x = [x0; sL; sR];
        xplus = ddmodel(x, b);
        plot([x0(1) xplus(1)], [x0(2) xplus(2)], 'k');
    end
end
