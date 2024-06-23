
% -- TASK 2 --
% Part (a): Making a nyquist plot and find a suitable Kf to stabilise G(s) 

% tf to create real-valued or complex-valued transfer function models, or to convert dynamic system models to transfer function form
G = tf([-0.0717,-1.684,-0.0853,-0.0622],[1,1.0604,-1.1154,-0.066,-0.0512]);

%nyquist(G);
%grid on
%title('Nyquist Plot of Transfer Function G(s)')

p = pole(G)
% Only one unstable pole (in RHP = 0.7282 + 0.0000i)
% To make G(s) stable, take one negative encirclement

Kf = -50;
% Find a Kf such that all poles of G_closed are in the RHP

% For G_closed to be stable, no poles of G_closed should be in the RHP

% Put a Kf and check the encirclement of point -1,0. To cancel out 1 pole in the RHP, we need 1 negative encirclement of (-1,0)
% This is only possible when Kf is some negative value
% The Kf should be between -1.1 to -1.4e36

L = G*Kf; %Open Loop Transfer Function
% Use open loop tf to find encirclements of -1,0 which will stbilise G(s)
pl = pole(L)
nyquist(L)
grid on
title('Nyquist Plot of Open-Loop Transfer Function Kf*G(s)')

Char_poly = 1+L 
% Use that Kf to get the characteristic polynomial and find its zeros
z_charpoly = zero(Char_poly)

% Poles of G_closed are zeros of the characteristic polynomial
G_closed = feedback(G,Kf);
p_closed = pole(G_closed)

% Part (b): Making a pole-zero chart and step response function graph to
% determine stability of G_closed
pzmap(G_closed)
grid on
title('Pole-Zero Chart of G closed')
% No poles in the RHP

step(G_closed)
grid on
title('Unit-Step Response of G closed')
% Step respose converges to a steady state

% From the graph directly, design criteria 2,3 are not satisfied
% For checking if steady state error is 0
%[y, t] = step(G_closed);
%SSE = abs(-0.0203 - y(end));

% Display the steady-state error using both methods
%disp('Steady-State Error:');
%disp(SSE);

% References: https://math.libretexts.org/Bookshelves/Analysis/Complex_Variables_with_Applications_(Orloff)/12%3A_Argument_Principle/12.02%3A_Nyquist_Criterion_for_Stability
