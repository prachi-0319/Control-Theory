close all
clc

% Given constants
g = 1;
k = 1;
epsilon = 1;
A = 1;
m = 1;
b = 1;
R = 1;

%% Task 1
num_points = 1000;
z_o1 = linspace(0.01*g, 0.99*g, num_points);
Vo = zeros(1, num_points);
max_real_part_eigenvalues = zeros(1, num_points);

% Calculate Vo for each value of z_o1
for i = 1:num_points
    Vo(i) = sqrt((2 * k * z_o1(i) * (g - z_o1(i))^2) / (epsilon * A));
    A_sys = [0, 1, 0;
             -k/m, -b/m, Vo(i) / (m * (g - z_o1(i)));
             Vo(i) / (R * (g - z_o1(i))), 0, - (g - z_o1(i)) / (epsilon * A * R)];
    eigenvalues = eig(A_sys);
    max_real_part_eigenvalues(i) = max(real(eigenvalues));
    
end

% Plot z_o1 against Vo
figure; % Create a new figure

% First subplot
subplot(2, 1, 1); 
plot(z_o1, Vo, 'b-', 'LineWidth', 2);
xlabel('z_{o1}');
ylabel('Vo');
title('Relationship between z_o1 and Vo');
grid on;

% Second subplot
subplot(2, 1, 2); 
plot(z_o1, max_real_part_eigenvalues, 'r-', 'LineWidth', 2);
xlabel('z_{o1} (m)');
ylabel('Max Real Part of Eigenvalues');
title('Plot of z_{o1} vs Max Real Part of Eigenvalues of A_{sys}');
grid on;

set(gcf, 'Position', [100,100,600,800])