% AI 3206 CONTROL SYSTEMS - PROJECT 1

% Team Number: 31
% Team Members:
    % Kuhu Sharma (SE21UCSE109)
    % Adit Rushil Potta (SE21UARI006)
    % Prachi Kansal (SE21UARI105)
    % Lanii Lakshitaa (SE21UARI073)

    
    
%% --- TASK 5: Simulation ---

Tsim = 1; % Defining time

% Taking inputs from the user 
amp = input('Enter the amplitude of the sinusoidal input (A): ');
omega = input('Enter the frequency of the sinusoidal input (omega): ');
alpha0 = input('Enter the non-zero initial condition alpha(0)');
alpha_deriv = input("Enter the non-zero initial condition alpha'(0)");

X0 = [0; alpha0; alpha_deriv; 0; 0];  % Initial states
abstol = 1e-5; % Absolute tolerance
reltol = 1e-5; % Relative tolerance
options = odeset('RelTol', reltol, 'AbsTol', abstol*ones(1,numel(X0)));

% Using the ODE45 Solver
[t, X] = ode45(@(t, X) state_dynamics_model(t, X, A, B, omega, Amp), [0,Tsim], X0);
y = C*transpose(X);

% Plotting the simulation results
plot(t, y, 'LineWidth', 1, 'Color', 'Blue');
grid on;

% Function to calculate derivatives
function [dX] = state_dynamics_model(t, X, A, B, omega, Amp)
    epsilon = Amp * sin(omega * t); % Second input
    
    % Cases for different time intervals
    if t<=1
        r = 10; % Value of the unit-step response r(t) at different time stamps
        u = [r; epsilon]; % Input states vector
    elseif (t>1) & (t<=12)
        r = -4;
        u = [r; epsilon];
    elseif (t>12) & (t<=16)
        r = -5;
        u = [r; epsilon];
    elseif t>16
        r = 8;
        u = [r; epsilon];
    end
   
    % Computing derivatives
    dX = A * X + B * u;
end



%% %% --- TASK 5: Simulation (The alternate TF approach) ---

Gs = tf([-0.0717 -1.684 -0.0853 -0.0622],[1 1.0604 -1.1154 -0.066 -0.0512]);

Kf = -50;
T = 0.45;
K = -7000;
s = tf('s');
C1 = tf(0.95, [1, 0]); % Transfer function for C1(s)
C2 = K * (1 + T*s);    % Transfer function for C2(s)
C = -C1 * C2; % Calculate the product C(s) = -C1(s) * C2(s)

x = (C*Gs/(1+C*Gs+Gs*Kf));
sys_tf_r = tf(x); % This the closed loop tf with step response r
%disp(sys_tf_r);

y = (Gs/(1+Gs*C+Gs*Kf));
sys_tf_epsilon = tf(y); % this is the one we get when we use the user 
% input i.e epsilon (Asin(wt))
%disp(sys_tf_epsilon);

t = 0:0.01:5; % Define time from 0 to 5 seconds with a step of 0.01 seconds

% Define the reference signal r(t)
r = zeros(size(t)); % Initialize r(t) as an array of zeros
r(t <= 0.1) = 10;             % from 0 to 0.1 sec
r(t > 0.1 & t <= 0.12) = -4;  % from 0.1 to 0.12 sec
r(t > 0.12 & t <= 0.16) = 5;  % from 0.12 to 0.16 sec
r(t > 0.16 & t <= 0.25) = 8;  % from 0.16 to 0.25 sec

sinusoidal_input = A * sin(omega * t); % Epsilon - an input

[y1, t1, x1] = lsim(sys_tf_r, r, t);
[y2, t2, x2] = lsim(sys_tf_epsilon, sinusoidal_input, t);

combined_output = y1 + y2; % Combine the outputs

% Plot the combined response
figure;
plot(t1, combined_output);
xlabel('Time (s)');
ylabel('Output');
title('System Response');
grid on;



%% ------------ END ---------------



