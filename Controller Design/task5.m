%% 
A = [0, 3.529074, 82.88648, 4.198466, 3.061484; 128, 198.6241, 4691.0554, 237.6265, 3.061484; 0, 1, 0, 1, 0; 0, 0, 1, 0, 0; 0,0,0,1,0];
% 5 X 5
B = [49.22, 0; 0, 1; 0, 0; 0, 0; 0, 0];
%5 X 2
C= [0 -0.0717 -1.6840 -0.0853 -0.0622];

Tsim = 1;
%t = linspace(0, Tsim, 501);

% Prompt the user for the sinusoidal input parameters
Amp = input('Enter the amplitude of the sinusoidal input (A): ');
omega = input('Enter the frequency of the sinusoidal input (omega): ');
%alpha0 = input('Enter the non-zero initial condition alpha(0)');
%alpha_deriv = input("Enter the non-zero initial condition alpha'(0)");

X0 = [0; 0.2; 0.003; 0; 0];  % State at t = 0 (Initial states). 5 X 1
% alpha - 2nd val of X0
% alpha dot - 3rd val of X0

abstol = 1e-5;
reltol = 1e-5;
options = odeset('RelTol', reltol, 'AbsTol', abstol*ones(1,numel(X0)));

[t, X] = ode45(@(t, X) state_dynamics_model(t, X, A, B, omega, Amp), [0,Tsim], X0);
%disp(X)
y = C*transpose(X);
plot(t, y, 'LineWidth', 1, 'Color', 'Blue');
grid on;

function [dX] = state_dynamics_model(t, X, A, B, omega, Amp)
    % Generate the sinusoidal input signal
    sino = Amp * sin(omega * t);
    %disp(t)
    %disp(sino)
    
    if t<=1
        r = 10;
        u = [r; sino];
    elseif (t>1) & (t<=12)
        r = -4;
        u = [r; sino];
    elseif (t>12) & (t<=16)
        r = -5;
        u = [r; sino];
    elseif t>16
        r = 8;
        u = [r; sino];
    end
    
    %disp(A)
    %disp(X)
    %disp(t)
    %disp(u)
    dX = A * X + B * u;
    disp(dX);
end