% AI 3206 CONTROL SYSTEMS - PROJECT 1

% Team Number: 31
% Team Members:
    % Kuhu Sharma (SE21UCSE109)
    % Adit Rushil Potta (SE21UARI006)
    % Prachi Kansal (SE21UARI105)
    % Lanii Lakshitaa (SE21UARI073)

    
    
%% --- TASK 1 ---

G = tf([-0.0717,-1.684,-0.0853,-0.0622],[1,1.0604,-1.1154,-0.066,-0.0512]);

% Plotting the unit step response
step(G);
grid on;

% Plotting the pole-zero chart
p = pole(G);
pzmap(G);
grid on;



%% % --- TASK 2 ---

% Part (a): Plotting a Nyquist plot and finding a suitable Kf to stabilise G(s) 
nyquist(G);
grid on;
title('Nyquist Plot of Transfer Function G(s)'); % Only one unstable pole (in RHP = 0.7282 + 0.0000i) -> one negative encirclement needed

Kf = -50; % Justification in the report

L = G*Kf; % Open Loop Transfer Function - use open loop tf to find encirclements of (-1,0) which will stbilise G(s)
pl = pole(L);
nyquist(L);
grid on;
title('Nyquist Plot of Open-Loop Transfer Function Kf*G(s)');

Char_poly = 1+L; % Characterisitc polynomial
z_charpoly = zero(Char_poly); % Poles of G_closed are zeros of the characteristic polynomial
G_closed = feedback(G,Kf);
p_closed = pole(G_closed);


% Part (b): Determine stability of G_closed
pzmap(G_closed);
grid on;
title('Pole-Zero Chart of G closed'); % No poles in the RHP

step(G_closed);
grid on;
title('Unit-Step Response of G closed'); % Step respose converges to a steady state

% Part (c): Were any of the design criteria satisfied?
[y, t] = step(G_closed); % For checking if steady state error is 0
SSE = abs(-0.0203 - y(end));
disp(SSE);

% From the graph directly, design criteria 2,3 were not satisfied



%% % --- TASK 3 ---

% Given C1(s) and C2(s) are such that the closed loop system C(s)*G_closed is stable
% C2(s) = Type 0 system
% For the system to have a steady state error of 0 when input in a unit step, it needs to be type 1 or above

K1 = 0.95;
K2 = -1; % C2 = K2
C1 = tf([K1],[1,0]); %C1 = K/s;
% C2 = tf([K2],[1]); 
TF_Fig4 = feedback(C1*K2*G_closed,1); % Closed-loop transfer function

t = 0:0.1:1000;
u = t;
[y,t,x] = lsim(TF_Fig4,u,t);
plot(t,y,'b',t,u,'m')
xlabel('Time (sec)')
ylabel('Amplitude')
title('Input-purple, Output-blue')
step(TF_Fig4) % Plotting the unit-step response graph 

% Finding the steady state error
[y, t] = step(TF_Fig4);
y_final = y(end);
y_desired = 1; % Assuming a step input of 1
SSE = abs(y_desired - y_final);
disp(SSE);



%% % --- TASK 4 ---

% Part (a): Bode plot of H_tilda(s)
H = -(C1 * G_closed);
margin(H); % Bode plot is used for open-loop transfer functions
% Gain margin: 49.4 dB at 10.2 rad/s
% Phase margin: 90 deg at 0.0193 rad/s


% Part (b): Is designing a proportional controller possible?
% For a proportional controller, C(s) = C1 * C2 = Kp
% To show that this will not satisfy the design criteria
K = 0.01;
Cs = K;
G_prop = feedback(Cs * G_closed, 1);
stepinfo(G_prop) % Printing step response information


% Part (c): Finding a possible structure of C2(s)
% Using a lead-compensator controller model for C2(s)
T = 1;
K = 1;
s = tf('s'); 
C2_structure = K * (1+T*s);

% Part (d): Tuning the parameters of C2(s) to satisfy the maximum overshoot and settling time criteria
T = 0.45;
K = -7000;

% Design Criterias:
% 1. The settling time for 2% settling accuracy is 0.05 seconds
% 2. The maximum percentage overshoot is not greater than 20%

C2 = K * (1+T*s);
G_closed = feedback(-C2 * H, 1);
step(G_closed);
stepinfo(G_closed); % To get the settling time and overshoot values
B = isstable(G_closed); % To check stability
disp(B);



%% --- TASK 5 ----

% Part (a): State space model of the controller C(s)
num = {[2835 6300] [-2785 -6300]}; % Considering a multi-input system with inputs r(t) and alpha
denom = {[1 0] [1 0]};
SS_controller = ss(tf(num, denom)); 


% Part (b): State space model of the F-16 aircraft G(s)
num_g= ([-0.0717,-1.684,-0.0853,-0.0622]);
denom_g = ([1,1.0604,-1.1154,-0.066,-0.0512]);
[A2,B2,C2,D2] = tf2ss(num_g,denom_g);


% Part (c): Combining the 2 state space models (calculations done manually)
A = [0, 3.529074, 82.88648, 4.198466, 3.061484; 128, 198.6241, 4691.0554, 237.6265, 3.061484; 0, 1, 0, 1, 0; 0, 0, 1, 0, 0; 0,0,0,1,0];
B = [49.22, 0; 0, 1; 0, 0; 0, 0; 0, 0];
C = [0 -0.0717 -1.6840 -0.0853 -0.0622];
D = [0];


% Part (d): Simulation of the state space model
% Code in a separate file simulator.m



%% ------------ END ---------------

