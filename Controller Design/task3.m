
% -- TASK 3 --
G = tf([-0.0717,-1.684,-0.0853,-0.0622],[1,1.0604,-1.1154,-0.066,-0.0512]);
Kf = -50;
G_closed = feedback(G,Kf);

% C1(s) = To meet the steady stateerror criteria
% C2(s) = To meet the settling time and maximum overshoot criteria

% Given C1(s) and C2(s) are such that the closed loop system C(s)*G_closed is stable
% C2(s) = Type 0 system
% Find C1(s) such that steady state error of the system is zero when r(t)
% is a step input

% To get a steady state error of 0 with r(t) as a step input, we need a
% system of type 1 or above.
% Type of a system = Number of poles at origin
% C2(s) is type 0, thus choose C1(s) such that C1(s)*C2(s) is type 1

% If we choose C1(s) as K/s, C = C1(s)*C2(s) will give us a type 1 system,
% for which the steady state error is zero when input is a step input

% Kdc = DC gain of a system = gain of output of the system when we apply a
% unit step input to it 
% Kdc = lim s-> 0 (s*Y(s)) = lim s->0 (G(s))
% If G(s) is type 0, Kdc = K = product of zeros/product of poles

% Also as G_closed is type 0, and we need ess to be zero for unit input ->
% essentially a PI controller -> C(s) = (KpS + Ki)/s (1 pole at s=0 -> type 1)
% C(s) = Kp + Ki/s

%K = dcgain(G)
%C = C1*C2;

p = pole(G_closed);
z = zero(G_closed);

%K = prod(z)/prod(p);
K1 = 0.95;
K2 = -1;

%C1 = K/s;
C1 = tf([K1],[1,0]);
%C2 = K;
C2 = tf([K2],[1]);

%L = minreal(G_closed*C1*C2)
%TF_Fig4 = minreal(L/1+L)

TF_Fig4 = feedback(C1*C2*G_closed,1);

t = 0:0.1:1000;
u = t;
[y,t,x] = lsim(TF_Fig4,u,t);
plot(t,y,'b',t,u,'m')
xlabel('Time (sec)')
ylabel('Amplitude')
title('Input-purple, Output-blue')

step(TF_Fig4)

% E(s) = R(s) - Y(s) = R(s) - C(s)*G_closed*E(s) => E(s) = R(s)/1+C*G_closed
% sE(s) = sR(s)/1+C*G_closed = poles of sE(s) = zeros of denom = all in lhp
ppole = pole(TF_Fig4);

[y, t] = step(TF_Fig4);
y_final = y(end);


y_desired = 1; % Assuming a step input of 1
SSE = abs(y_desired - y_final);

step(TF_Fig4);
disp(['Steady-state error (SSE): ', num2str(SSE)])

% reference: https://ctms.engin.umich.edu/CTMS/index.php?aux=Extras_ess1


