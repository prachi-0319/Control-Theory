close all
clear all
clc

%% System parameters
m = 1;
k = 1;
b = 1;
epsilon = 1;
A = 1;
g = 1;
R = 1;

%% User inputs
% Tsim: Simulation time (in seconds).
% x_o: Position of the moving plate at equilibrium.
% Z_i: Initial state of the system.
% x_r: Desired position of the moving plate (the reference input).

Tsim = 50;
x_o = 0.1; % Keep x_o between 0 and 0.8 for safety issues (top and bottom plate coming in contact with each other).
delta = 0.05; % 5% variation

% Z_i = ?; % Write a SMALL piece of code to RANDOMLY set initial Z_i around equilibrium point x_o.
Vo = (sqrt(2*k*x_o)*(g-x_o))/(epsilon*A);
zo_3 = epsilon*A*Vo / (g-x_o);

% Generate random initial state
Zi = [x_o*(1 + delta*(2*rand()-1)); 0; zo_3*(1 + delta*(2*rand()-1))];

x_r = 0.12; % Choose x_r around x_o.

%% Code to simulate the system
options = odeset('RelTol', 1e-8, 'AbsTol', (1e-8)*ones(1,numel(Zi)));
[t, Z] = ode45(@(t,Z) state_dynamics_model(t, Z, m, k, b, epsilon, A, g, R, x_r), [0 Tsim], Zi, options);
x = Z(:,1); % This is the plate position.

%% Plotting the system output (position of the moving plate) with respect to time
plot(t, x, LineWidth=1, Color='black');
hold on;
plot(t, x_r*ones(1,numel(t)), Color='blue');
hold off;
grid on;
xlabel('Time $t$', Interpreter='latex', FontSize=14);
legend('Position $x\left(t\right)$', 'Desired Position $x^{r}$', Interpreter='latex', FontSize=14);


function [dZ] = state_dynamics_model(t, Z, m, k, b, epsilon, A, g, R, xr)
% x_r: Desired position of the moving plate (the reference input).

z1 = Z(1); % Position of the plate.
z2 = Z(2); % Velocity of the plate.
z3 = Z(3); % Charge on the the plate.
 
% Controller dynamics
% Vs = ?; Open loop controller equation here.
Vs = sqrt((2*k*xr*(g-xr)^2) / (epsilon*A));

% System dynamics
dz1 = z2;
dz2 = -(k/m)*z1 - (b/m)*z2 + (z3^2)/(2*epsilon*A*m);
dz3 = -z3*(g-z1)/(epsilon*A*R) + Vs/R;

dZ = [dz1; dz2; dz3];

end