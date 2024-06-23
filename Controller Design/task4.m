% Design C2(s) satisfying maximum overshoot and settling time criteria
% Consider H(s) = C1(s) * Gn
% Design C2(s) using Bode plot of H(s)
% Use -H(s) and subsequently, use -C2(s)


num_coeff = [-0.0717 -1.684 -0.0853 -0.0622];
denom_coeff = [1 1.0604 -1.1154 -0.066 -0.0512];

Gs = tf(num_coeff, denom_coeff);

Kf = -50;
Gn = feedback(Gs, Kf);

% Choose K according to the Bode plot changes
%K_C1 = 1;
%C1 = tf(K_C1, [1, 0]);

% C1 = K/s
C1 = tf(0.95, [1, 0]);  %ki/s

% Taking C2 as compensator

T = 0.45;
K = -7000;
s = tf('s');

%C2 = K * (1+a*T*s)/(1+T*s);
C2 = K * (1+T*s);

H = -(C1 * Gn);

% Bode plot of -H(s)
% Bode plot is used for open-loop transfer functions
margin(H);


% Using Bode plot to find C2(s)
% (b) If we take a proportional controller:
% C(s) = C1 * C2 = Kp
% Now show that this will not satisfy design criteria
% K = 5;
% Cs = K;
% G_closed = feedback(Cs * Gn, 1);
% stepinfo(G_closed)



% We know the formula for PI controller:
% C(s) = Kps + Ki/s
% Find damping ratio corresponding to oveershoot of 20%
% Damping ratio = 0.4559
% Phase margin should be atleast 53.9185
% Bandwidth frequency >= 175.477

% C2(s) is a Type-0 system
% Using Bode Plot of -H(s), find C2(s)
% Meet maximum overshoot and settling time criteria
% Closed loop system in Figure 5 must be stable for that
% Design Criterias:
% 1. The settling time for 2% settling accuracy is 0.05 seconds
% 2. The maximum percentage overshoot is not greater than 20%
% Maximum overshoot decreases with increasing phase margin
% Settling time decreases with increase in Gain Crossover Frequency
% Choosing K is also an important step as K->Gain
% K is inversely proportional to Phase Margin
% K is proportional to Gain Crossover Frequency

% Maximum overshoot needs to decrease, so increase PM
% Subsequently, decrease K

% Find C2 s.t G_closed is stable irrespective of how high K is


%Poles = pole(G_closed);
%disp(Poles);

% Check stability of closed-loop system regardless of gain
%L = -C2 * H;
%G_closed = tf(L, [0, 1+L]);
% series = minreal(C2 * H);
G_closed = feedback(-C2 * H, 1);
step(G_closed);

stepinfo(G_closed)

%Poles = pole(G_closed);
%disp(Poles);

B = isstable(G_closed);
disp(B);




